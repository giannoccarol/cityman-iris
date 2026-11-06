# cityman-iris engine

Motore di riconoscimento dell'iride per lo scanner IR del Lumia 950 XL (cityman): camera Toshiba
`0x0861`, 1920×1088 RAW10 monocromatica, LED 850 nm sul canale 0 del flash PMI8994 (contesto in
`Android 950 XL/IRIS-CITYMAN-2026-11-06.md`). C++17 senza dipendenze esterne. Compila come libreria
vendor Android (`libcitymaniris_engine`) e su host con g++ o clang.

**Stato (6/11/2026).**
- Libreria, API C, test unitari, CLI e `Android.bp` sono pronti. Su host il motore compila senza
  avvisi con g++ 16 e clang 23 (`-Wall -Wextra -Wshadow -Wconversion`) e passa i test, anche sotto
  ASan/UBSan e con `-fno-exceptions`.
- **Non è stato provato su immagini di occhi.** I test su immagini usano fantocci geometrici
  (dischi concentrici) e "rubber sheet" casuali. Il generatore sintetico di occhi previsto non è
  stato completato: il file locale `tests/synth_eye.*` è troncato ed escluso dal repository. Mancano
  quindi le curve FAR/FRR su occhi.
- Soglie e parametri vanno tarati sulle prime catture reali.
- La build Android non è stata lanciata.

## Struttura

| Percorso | Contenuto |
|---|---|
| `include/iris_engine.h` | API C pubblica (stabile, versionata) |
| `src/image.cpp` | conversione ingresso (GRAY8, GRAY16, MIPI RAW10, ROI, autoguadagno), blur, bilineare |
| `src/segment.cpp` | candidati pupilla, contorno pupilla, limbo, palpebre, maschere |
| `src/encode.cpp` | rubber sheet, FFT, log-Gabor, codice e maschere, distanza di Hamming |
| `src/iris_api.cpp` | pipeline, cancelli di qualità, match/verify/enroll, serializzazione |
| `tests/unit_tests.cpp` | test host (82 controlli) e tempi |
| `tools/iris_cli.cpp` | CLI per catture reali (extract, match, enroll, overlay di debug) |
| `CMakeLists.txt`, `Android.bp`, `libcitymaniris_engine.map.txt` | build host e vendor; esporta solo `iris_*` |

## API

```c
iris_config cfg;
iris_config_default(&cfg);            /* poi modificare i campi, es. cfg.min_iris_radius */
iris_engine *eng;
iris_engine_create(&cfg, &eng);

iris_image img = {0};
img.data = raw10; img.width = 1920; img.height = 1088;
img.stride = 2400; img.format = IRIS_PIX_RAW10_MIPI;   /* o GRAY8 / GRAY16 con bits=10 */
iris_template *probe;
iris_quality q = { .struct_size = sizeof(q) };
iris_status st = iris_extract_image(eng, &img, &probe, &q);
/* st != IRIS_OK -> probe == NULL; q spiega il perché (sfocato, troppo piccolo, occluso, ...) */

iris_template *enrolled;               /* da 3+ campioni dello stesso occhio */
iris_enroll(eng, samples, n, &enrolled, &info);
int32_t ok; float score;
iris_verify(eng, probe, enrolled, &ok, &score);

size_t n; iris_template_serialize(enrolled, NULL, 0, &n);   /* dimensione */
/* ... buffer di n byte ... */ iris_template_serialize(enrolled, buf, n, &n);
iris_template_destroy(probe); iris_template_destroy(enrolled); iris_engine_destroy(eng);
```

**Contratti.**
- Gli oggetti sono opachi e li libera chi li riceve: `*_destroy`. Nessuna allocazione resta
  nascosta e nessun puntatore ai buffer del chiamante viene trattenuto.
- **Thread safety:** l'engine è immutabile dopo la creazione. `extract`, `match` ed `enroll` si
  possono chiamare in parallelo sullo stesso engine (c'è un test con 8 thread).
- Con `-fexceptions` (impostato in `Android.bp`) una `bad_alloc` diventa `IRIS_E_NO_MEMORY`.
  Senza eccezioni, l'esaurimento della memoria termina il processo, come nel resto di Android.

**Codici di qualità** (`IRIS_Q_*`, restituiti da extract, con `iris_quality` compilata comunque):

| Codice | Significato |
|---|---|
| `IMAGE_TOO_SMALL` | immagine più piccola dell'occhio minimo cercabile |
| `UNDEREXPOSED` | 99,5° percentile sotto il 4% del fondo scala |
| `OVEREXPOSED` | oltre il 20% dell'anello dell'iride saturo |
| `NO_EYE` | nessuna pupilla |
| `NO_IRIS` | pupilla trovata, ma non il confine iride/sclera |
| `BAD_GEOMETRY` | rapporto pupilla/iride fuori da 0,1–0,85, o pupilla fuori dall'iride |
| `IRIS_TOO_SMALL` | raggio sotto `min_iris_radius`: avvicinarsi |
| `OUT_OF_FRAME` | più di 1/4 del limbo fuori dall'immagine o dalla ROI |
| `BLURRED` | σ di sfocatura oltre 0,3 × la lunghezza d'onda più corta a metà iride |
| `OCCLUDED` | meno del 35% dei campioni validi (palpebre, ciglia, riflessi) |
| `LOW_TEXTURE` | geometria valida ma senza tessitura codificabile (es. superficie piatta stampata) |

`iris_quality` riporta inoltre centri e raggi (in coordinate dell'immagine d'ingresso), σ di
sfocatura, contrasto di limbo e pupilla, copertura palpebrale, frazione di bit utili, presenza del
glint del LED ed esposizione. Servono alla UI per guidare l'utente.

## Pipeline e scelte

1. **Ingresso.**
   - Formati: GRAY8, GRAY16 (`bits`, di solito 10) e MIPI RAW10 impacchettato (4 pixel in 5
     byte), con ROI opzionale.
   - Autoguadagno con LUT: il 99,5° percentile va a 230, con guadagno massimo 16×. Così un 10 bit
     scuro (esposizione di 350 µs) non perde livelli.
2. **Candidati pupilla.**
   - Fotogramma ridotto con media a blocchi (fattore dato dal raggio minimo della pupilla), poi da
     4 a 8 soglie scure: percentili e livelli assoluti sopra il minimo.
   - L'apertura 3×3 per staccare le ciglia si fa una volta sola, come chiusura in scala di grigi:
     per decomposizione a soglie, l'apertura binaria di `{c<T}` è `{chiusura(c)<T}`.
   - Componenti connesse per run con union-find: il costo segue il numero di run, non l'area, e gli
     sfondi scuri tipici del LED vicino restano economici.
   - Riempimento dei buchi, cioè il glint del LED dentro la pupilla.
   - Punteggio: circolarità², contrasto con l'anello attorno, frazione scura dell'anello e bonus
     glint. Si tengono fino a 4 ipotesi; i dischi concentrici di taglia diversa restano ipotesi
     separate, per esempio l'iride intera sotto una soglia alta.
3. **Contorno pupilla.**
   - Ricerca radiale del massimo gradiente buio→chiaro a piena risoluzione, saltando i raggi che
     toccano riflessi.
   - Fit robusto del cerchio (Kasa), poi contorno di Fourier di ordine 2 (Daugman 2007), che copre
     anche le pupille ellittiche.
   - Dal profilo medio del bordo si stima la **sfocatura**: per un gradino gaussiano la pendenza
     massima vale H/(σ√2π). Sui fantocci σ=1,5 → 1,56 e σ=3 → 3,04.
4. **Limbo.**
   - Operatore integro-differenziale di Daugman sugli archi laterali (−30°…+45° e 135°…210°), che
     le palpebre coprono di rado.
   - Raggio vincolato tra `r_pupilla_max + 9` e `r_pupilla/0,12`. Le 9 px tengono gli archi fuori
     dal bordo sfocato della pupilla: senza questo vincolo il fit agganciava la pupilla.
   - Due stadi: profilo radiale dal centro della pupilla con i 3 picchi migliori, poi i centri
     attorno a quei raggi, poi discesa per coordinate fino a 0,25 px.
5. **Palpebre.** Ricerca integro-differenziale di parabole sopra e sotto la pupilla, con le sonde
   lontane ±7 px dai bordi di pupilla e limbo. Accettata se il gradino supera 0,5 × il contrasto
   del limbo.
6. **Maschere.**
   - Riflessi: ≥245 dopo l'autoguadagno, dilatati di 2 px. Sclera e pelle arrivano a ~230 e non
     vanno mascherate: un primo errore qui svuotava gli archi del limbo.
   - Ciglia: più scure di mediana − 3·MAD e della metà tra pupilla e iride.
   - Pixel fuori dall'immagine.
7. **Normalizzazione.** Rubber sheet di Daugman 64×512 tra il contorno della pupilla e il cerchio
   del limbo, raggio normalizzato 0,02–0,92.
8. **Codifica.**
   - Smoothing gaussiano radiale con maschera (σ=2 righe), poi log-Gabor 1D lungo l'angolo (Field
     1987, Masek 2003) su due scale: λ = 32 e 64 colonne, σ/f₀ = 0,5.
   - Fase quantizzata a 2 bit (Daugman). Codice 16×256 × 2 filtri × 2 bit = 16384 bit, cioè una
     parola da 64 bit per colonna (lo shift rotazionale costa un offset di indice).
   - Maschera: campioni validi in una finestra di ±λ/4, più i **fragile bits** (Hollingsworth,
     Bowyer, Flynn 2009): il 15% delle componenti più vicine a zero.
9. **Confronto.**
   - Distanza di Hamming frazionaria sui bit validi in comune, minima su ±10 colonne del codice
     (±14°) e su tutte le coppie di codici.
   - Normalizzazione di Daugman 2004: `HDn = 0,5 − (0,5 − HD)·√(n/n_ref)`, con `n_ref` = 70% dei bit
     del codice (`reference_bits = 0` vuol dire automatico, <0 restituisce la HD grezza).
   - Se la sovrapposizione è sotto il 15% → `IRIS_E_NOT_ENOUGH_OVERLAP`.
10. **Enrollment.** Medoide degli N campioni. Si scartano quelli oltre `enroll_consistency`
    (0,32) rispetto al medoide e si tengono fino a 5 codici, i più centrali. Restituisce
    `IRIS_E_ENROLL_INCONSISTENT` con meno di 3 campioni coerenti. Due occhi diversi vanno in due
    template.
11. **Template v1.**
    - Formato little-endian: magic `CIRT`, versione, layout del codice, `encoder_id`, poi i codici e
      un CRC-32 finale.
    - `encoder_id` è un hash dei parametri che cambiano il significato dei bit: template con layout
      diverso danno `IRIS_E_INCOMPATIBLE`.
    - Dimensione: 4140 byte con 1 codice, 20556 byte con 5.

**Default** (`iris_config_default`):
- raggio iride 40–130 px;
- soglia di match 0,36;
- `max_blur_ratio` 0,30;
- `min_usable_fraction` 0,35.

Sono valori di partenza, non tarati.

**Risoluzione.** Col FOV orizzontale ~40° del report (§5.3), il diametro dell'iride (12 mm) è di
circa 127 px a 25 cm e 106 px a 30 cm. 100/150/200 px corrispondono a circa 32/21/16 cm. Il
minimo accettato (80 px di diametro) arriva fino a ~40 cm. ISO/IEC 19794-6 considera minimo un
diametro di 100 px e buono uno ≥150 px.

## Build

```sh
CM=/home/lorenzo/Documenti/Projects/Jail/android-cityman/los23/prebuilts/cmake/linux-x86/bin/cmake
$CM -S engine -B /tmp/iris-build -DCMAKE_CXX_COMPILER=clang++
$CM --build /tmp/iris-build -j8 && /tmp/iris-build/iris_unit_tests
```

Android: `engine/Android.bp` definisce `libcitymaniris_engine` (cc_library_shared, vendor, header
esportati da `include/`, solo simboli `iris_*`), più `libcitymaniris_engine_static`,
`citymaniris_cli` e `citymaniris_engine_tests` (non in `PRODUCT_PACKAGES`). **Non compilato in AOSP.**

## CLI

```sh
iris_cli extract frame.pgm -o a.tpl --debug a.ppm            # PGM 8/16 bit
iris_cli extract dump.raw --raw10 1920 1088 2400 -o a.tpl    # RAW10 MIPI (stride opzionale)
iris_cli extract dump.raw --raw16 1920 1088 10 --roi 800 300 600 500
iris_cli match a.tpl b.tpl
iris_cli enroll user.tpl s1.tpl s2.tpl s3.tpl s4.tpl
```

`--debug` scrive un PPM con il ritaglio attorno all'occhio:
- pupilla in verde, limbo in rosso, palpebre in blu, candidati in giallo;
- sotto, il rubber sheet, con in viola i campioni mascherati.

È il primo strumento da usare sulle catture del telefono.

## Risultati misurati

Host: Intel Core Ultra 7 155H (P-core fino a 4,8 GHz), 1 thread, `-O3`. La macchina era condivisa
con altre sessioni, quindi i tempi sono riportati come intervalli tra più esecuzioni.

**Segmentazione su fantocci** (disco pupilla 20, anello iride 95, sfondo 170 in 8 bit, bordi
antialias, glint saturo):

| r pupilla / r iride (px) | errore centro pupilla | r pupilla stimato | errore centro iride | r iride stimato | glint | stato |
|---|---|---|---|---|---|---|
| 20 / 50 | 0,00 | 19,99 | 0,00 | 49,99 | sì | LOW_TEXTURE |
| 30 / 75 | 0,01 | 29,97 | 0,01 | 74,97 | sì | LOW_TEXTURE |
| 45 / 100 | 0,00 | 44,98 | 0,00 | 99,98 | sì | LOW_TEXTURE |
| 18 / 60 | 0,00 | 17,98 | 0,00 | 60,00 | sì | LOW_TEXTURE |

`LOW_TEXTURE` è il risultato atteso: l'anello è uniforme. Gli altri cancelli funzionano: iride di
raggio 28 → `IRIS_TOO_SMALL`, iride tagliata dal bordo → `OUT_OF_FRAME`, fotogramma uniforme →
`NO_EYE`, fotogramma scuro → `UNDEREXPOSED`.

**Sfocatura stimata dal bordo della pupilla:**

| σ applicata | σ stimata |
|---|---|
| 0 (bordo antialias) | 0,38–0,44 |
| 1,5 | 1,56 |
| 3,0 | 3,04 |

**Codifica e confronto su rubber sheet casuali.** Sono 30 campi casuali periodici, cioè somme di
sinusoidi più rumore: non sono iridi. Il probe è ruotato di −7…+7 colonne del codice e ha rumore
indipendente da quello del riferimento:

| σ rumore | HD grezza genuina media / max | score genuino max | score impostore medio / sd / min |
|---|---|---|---|
| 8 | 0,000 / 0,000 | 0,000 | 0,455 / 0,028 / 0,341 |
| 20 | 0,001 / 0,003 | 0,000 | 0,455 / 0,027 / 0,344 |
| 35 | 0,011 / 0,024 | 0,003 | 0,456 / 0,027 / 0,350 |
| 50 | 0,034 / 0,062 | 0,048 | 0,457 / 0,026 / 0,354 |

Questi campi hanno troppa energia a bassa frequenza, perciò la parte genuina è ottimistica. Il test
verifica soltanto che la rotazione venga recuperata esattamente, che le maschere funzionino e che
gli impostori stiano intorno a 0,45. I gradi di libertà degli impostori, p(1−p)/σ², sono circa 340
qui, contro 249 nei dati reali di Daugman. **Non sono una stima di FAR/FRR.**

**Tempi:**

| Operazione | clang 23 | g++ 16 |
|---|---|---|
| extract, fantoccio 640×480 (iride r 65) | 7,9–11,9 ms | 8,7–10,9 ms |
| extract, fantoccio 1920×1088 (iride r 65) | 11,6–18,4 ms | 13,2–15,5 ms |
| solo front end 1920×1088 senza occhio, 8 bit / 10 bit | 5,3–7,8 ms | 6,1–8,3 ms |
| encode (64×512 → 16384 bit) | 1,6–3,0 ms | 1,7–1,8 ms |
| match 1 codice vs 1 codice, ±10 shift | 0,013–0,033 ms | 0,015–0,019 ms |

Ottimizzazioni misurate sul front end a 1920×1088:
- apertura separabile: 83 → 42 ms;
- etichettatura per run: 26 → 7 ms con sfondo scuro;
- chiusura in scala di grigi unica e istogrammi interlacciati: extract su fotogramma intero da 28,8
  a 11,7 ms (clang).

**Stima Cortex-A57 a 1,6 GHz.** Considero un fattore da 7 a 9 rispetto a questo host (clock 3×, IPC
2,5–3×, banda di memoria inferiore):
- extract su fotogramma intero 80–140 ms, obiettivo < 150 ms rispettato ma al limite;
- con la ROI sull'occhio (640×480) 55–100 ms;
- verify contro 5 codici < 1 ms.

Su fotogrammi reali con tessitura si possono provare fino a 3 ipotesi di pupilla, ciascuna da
~3–5 ms host in più. **Non misurato sul telefono.** Non c'è una toolchain aarch64 con sysroot su
questa macchina: l'unica verifica è la compilazione clang con `-fno-exceptions -fno-rtti`.

**FAR teorica** di un confronto singolo (`iris_binomial_far`, modello binomiale di Daugman):

| Soglia HD | 249 gradi di libertà | 340 gradi di libertà |
|---|---|---|
| 0,30 | 6,6e-11 | 5,8e-14 |
| 0,32 | 4,0e-9 | 7,6e-12 |
| 0,34 | 1,6e-7 | 1,2e-9 |
| 0,36 | 4,0e-6 | 1,1e-7 |
| 0,38 | 6,7e-5 | 5,1e-6 |

Va moltiplicata per il numero di shift e di codici confrontati (21 × 5). Il dato vero sarà quello
delle catture reali.

## Limiti

- **Nessuna prova su occhi, né sintetici né reali**: l'accuratezza della segmentazione con
  ciglia, palpebre, riflessi e tessitura reale non è verificata. FAR e FRR restano da misurare.
- Limbo modellato come cerchio. Pupilla con contorno di Fourier di ordine 2. Con lo sguardo molto
  di lato servirebbe un modello ellittico anche per il limbo.
- Palpebre parabole, ciglia per soglia: nessun modello di ciglia diffuse (Kong & Zhang).
- Nessun anti-spoofing oltre a `LOW_TEXTURE` e al flag del glint. Per la liveness servono il
  confronto LED acceso/spento e il micro-movimento della pupilla, nell'HAL.
- Il template sta in `/data/vendor`, senza TEE: al massimo **Classe 1 (convenience)**, come nel
  report §5.2.
- Default (soglie, `reference_bits`, `max_shift`, lunghezze d'onda) scelti a priori.
- Un occhio per template: la scelta dell'occhio (ROI o miglior candidato) spetta all'HAL.

## Dati reali: opzioni

Nessun dataset è stato scaricato.

- **Catture proprie col telefono** (consigliato). L'utente, ed eventualmente volontari consenzienti,
  con `iris_cli --debug` per la taratura. Restano locali e non vanno pubblicate: sono dati
  biometrici.
- **Dataset pubblici NIR**, da verificare e accettare dall'utente prima di scaricarli. In genere
  chiedono registrazione e un accordo d'uso non commerciale e vietano la ridistribuzione:
  - CASIA-Iris v4 (CASIA, Pechino);
  - ND-IRIS-0405 / ND-CrossSensor (Università di Notre Dame, licenza tramite accordo);
  - IITD Iris (IIT Delhi);
  - UBIRIS v2 (luce visibile, meno adatto).

  Servirebbero solo per tarare le soglie su host, non per includerli nel repository.

## Crediti e fonti dei metodi

Il codice è scritto da zero e nessun sorgente esterno è stato copiato o consultato. Metodi:

- **J. Daugman.**
  - "High confidence visual recognition of persons by a test of statistical independence", IEEE
    TPAMI 15(11), 1993: operatore integro-differenziale, codice di fase a 2 bit, Hamming.
  - "How iris recognition works", IEEE TCSVT 14(1), 2004: normalizzazione degli score, modello
    binomiale e gradi di libertà, focus.
  - "New methods in iris recognition", IEEE TSMC-B 37(5), 2007: contorni attivi di Fourier.
- **L. Masek**, "Recognition of human iris patterns for biometric identification", tesi, University
  of Western Australia, 2003: rubber sheet discreto, log-Gabor 1D, maschere di rumore.
- **D. J. Field**, "Relations between the statistics of natural images and the response properties
  of cortical cells", JOSA A 4(12), 1987: filtro log-Gabor.
- **K. Hollingsworth, K. Bowyer, P. Flynn**, "The best bits in an iris code", IEEE TPAMI 31(6),
  2009: fragile bits.
- **R. P. Wildes**, "Iris recognition: an emerging biometric technology", Proc. IEEE 85(9), 1997:
  alternativa Hough e palpebre parametriche.
- **I. Kåsa**, "A circle fitting procedure and its error analysis", IEEE Trans. Instrum. Meas. 25,
  1976: fit algebrico del cerchio.
- **Decomposizione a soglie degli operatori morfologici** (Serra; Maragos e Ziff, IEEE TPAMI
  1990) ed **etichettatura per run** (Rosenfeld e Pfaltz 1966; He, Chao, Suzuki, IEEE TIP 2008).
- Riferimenti citati nel report ma **non usati nel codice**:
  - **OSIRIS** (Télécom SudParis), licenza propria non verificata;
  - **open-iris** (Worldcoin Foundation), MIT secondo PyPI.

  Restano utili per confrontare FAR e FRR su PC.
- ISO/IEC 19794-6 e 29794-6: requisiti di risoluzione e qualità dell'immagine dell'iride.
- Hardware e contesto: report `IRIS-CITYMAN-2026-11-06.md`, con i crediti per driver, DCC e DSDT
  (Android4Lumia950, LumiaWoA, TechInsights).
