/*
 * cityman-iris engine: iris segmentation, encoding and matching.
 *
 * Small, stable C API. All objects are opaque and owned by the caller:
 *   - every iris_engine created with iris_engine_create() is released with iris_engine_destroy();
 *   - every iris_template returned by the library is released with iris_template_destroy().
 * The library never keeps references to caller buffers after a call returns.
 *
 * Thread safety: an iris_engine is immutable after creation, so any number of threads may call
 * iris_extract*(), iris_match*(), iris_enroll() concurrently on the same engine. Templates are
 * read-only for every function except iris_template_destroy().
 *
 * Scores are fractional Hamming distances in [0, 1]: lower means more similar; unrelated irises
 * score around 0.45-0.5.
 */
#ifndef CITYMAN_IRIS_ENGINE_H
#define CITYMAN_IRIS_ENGINE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__clang__)
#define IRIS_API __attribute__((visibility("default")))
#else
#define IRIS_API
#endif

/* Bumped on any incompatible change of this header. */
/* 2: iris_context and iris_extract_ex (search hint, reusable buffers). */
#define IRIS_ENGINE_API_VERSION 2
/* Version written in serialized templates (see iris_template_serialize). */
#define IRIS_TEMPLATE_FORMAT_VERSION 1

typedef enum iris_status {
    IRIS_OK = 0,

    /* Call errors */
    IRIS_E_INVALID_ARGUMENT = 1,
    IRIS_E_NO_MEMORY = 2,
    IRIS_E_BUFFER_TOO_SMALL = 3,  /* serialize: *size_out holds the size needed */
    IRIS_E_BAD_TEMPLATE = 4,      /* truncated, corrupt (CRC) or malformed serialized template */
    IRIS_E_TEMPLATE_VERSION = 5,  /* serialized with an unsupported format version */
    IRIS_E_INCOMPATIBLE = 6,      /* templates or engine use a different code layout */
    IRIS_E_INTERNAL = 7,

    /* Capture quality: iris_extract() rejected the image. iris_quality is still filled. */
    IRIS_Q_IMAGE_TOO_SMALL = 100, /* image smaller than the smallest searchable eye */
    IRIS_Q_UNDEREXPOSED = 101,    /* too dark: raise exposure / LED current */
    IRIS_Q_OVEREXPOSED = 102,     /* iris saturated */
    IRIS_Q_NO_EYE = 103,          /* no pupil-like dark blob found */
    IRIS_Q_NO_IRIS = 104,         /* pupil found but no iris/sclera boundary */
    IRIS_Q_BAD_GEOMETRY = 105,    /* implausible pupil/iris ratio or pupil outside iris */
    IRIS_Q_IRIS_TOO_SMALL = 106,  /* iris radius below config.min_iris_radius: move closer */
    IRIS_Q_OUT_OF_FRAME = 107,    /* iris partly outside the image (or the ROI) */
    IRIS_Q_BLURRED = 108,         /* out of focus or motion blur */
    IRIS_Q_OCCLUDED = 109,        /* eyelids/lashes/reflections leave too few usable bits */
    IRIS_Q_LOW_TEXTURE = 110,     /* iris visible but with no usable texture (flat, e.g. a print) */

    /* Matching / enrollment */
    IRIS_E_NOT_ENOUGH_OVERLAP = 200, /* masks overlap too little for a decision */
    IRIS_E_ENROLL_INCONSISTENT = 201 /* not enough mutually consistent samples */
} iris_status;

/* Pixel formats accepted by iris_extract_image(). */
typedef enum iris_pixel_format {
    IRIS_PIX_GRAY8 = 0,       /* 1 byte per pixel */
    IRIS_PIX_GRAY16 = 1,      /* uint16 per pixel (host endianness), `bits` significant LSBs (e.g. 10) */
    IRIS_PIX_RAW10_MIPI = 2   /* MIPI CSI-2 RAW10 packed: 4 pixels in 5 bytes */
} iris_pixel_format;

typedef struct iris_image {
    const void *data;
    int32_t width;
    int32_t height;
    int32_t stride;   /* bytes per row; 0 = tightly packed */
    int32_t format;   /* iris_pixel_format */
    int32_t bits;     /* GRAY16 only: significant bits (1..16); 0 = 10 */
    /* Optional region of interest (in pixels); roi_w or roi_h == 0 means the whole image. */
    int32_t roi_x, roi_y, roi_w, roi_h;
} iris_image;

/*
 * Engine configuration. Always initialize with iris_config_default() and then change fields.
 * The fields marked [layout] define the code layout: templates are only comparable when they
 * were produced with identical layout values.
 */
typedef struct iris_config {
    uint32_t struct_size;          /* set by iris_config_default() */

    /* Geometry, in pixels of the input image */
    float min_iris_radius;         /* 40: smaller irises are rejected (IRIS_Q_IRIS_TOO_SMALL) */
    float max_iris_radius;         /* 130: search limit */

    /* Normalization and encoding [layout] */
    uint16_t norm_rows;            /* 64: radial samples of the rubber sheet */
    uint16_t norm_cols;            /* 512: angular samples, power of two */
    uint16_t code_rows;            /* 16: code rows (norm_rows must be a multiple) */
    uint16_t code_cols;            /* 256: code columns (norm_cols must be a multiple) */
    uint16_t n_filters;            /* 2: number of log-Gabor scales (1 or 2) */
    uint16_t reserved0;
    float wavelength[2];           /* 32, 64: log-Gabor centre wavelengths, in norm columns */
    float sigma_on_f;              /* 0.5: log-Gabor bandwidth (sigma/f0) */
    float radial_sigma;            /* 2.0: Gaussian smoothing across rows, in norm rows */
    float fragile_fraction;        /* 0.15: weakest responses masked as fragile bits */

    /* Quality gates (iris_extract) */
    float max_blur_ratio;          /* 0.30: blur sigma / texture wavelength at mid-iris */
    float min_usable_fraction;     /* 0.35: minimum fraction of valid code bits */
    float min_limbus_contrast;     /* 3.0: grey levels (8-bit scale) of the iris/sclera step */

    /* Matching */
    uint16_t max_shift;            /* 10: rotation search, +/- code columns */
    uint16_t reserved1;
    float match_threshold;         /* 0.36: accept when score <= threshold (calibrate on real data!) */
    float reference_bits;          /* 0: Daugman score normalization reference (0 = auto, 70% of the
                                      code bits; < 0 = report the raw Hamming distance) */
    float min_overlap_fraction;    /* 0.15: minimum fraction of commonly valid bits */

    /* Enrollment */
    uint16_t enroll_min_samples;   /* 3 */
    uint16_t enroll_max_codes;     /* 5 */
    float enroll_consistency;      /* 0.32: max score of a sample against the medoid */

    /* Candidate acceptance (since API 2). A real eye under the IR LED has a substantial dark
       pupil inside the iris; a tiny dark speck (hair, nostril) with a large textured "iris"
       around it does not. min_pupil_iris_ratio rejects candidates whose pupil is too small a
       fraction of the iris, the main cause of hair/skin false positives on close IR frames. */
    float min_pupil_iris_ratio;    /* 0.18 */
} iris_config;

/* Measured capture quality, filled by iris_extract*() even when the image is rejected. */
typedef struct iris_quality {
    uint32_t struct_size;          /* set by the caller to sizeof(iris_quality) */
    int32_t status;                /* same value returned by the call */
    float overall;                 /* 0..100 summary (0 when rejected) */
    float focus;                   /* 0..100, 50 at the max_blur_ratio gate */
    float blur_sigma;              /* estimated blur sigma in pixels (pupil edge) */
    float usable_fraction;         /* valid code bits / total */
    float eyelid_coverage;         /* fraction of the iris ring hidden by eyelids */
    float exposure;                /* 99.5th percentile of the input, 0..1 of full scale */
    float pupil_x, pupil_y, pupil_r;   /* input-image coordinates; r = mean radius */
    float iris_x, iris_y, iris_r;
    float pupil_iris_ratio;
    float limbus_contrast;         /* grey levels (8-bit scale) */
    float pupil_contrast;
    int32_t glint_found;           /* LED corneal reflection seen inside/near the pupil */
    int32_t eye_found;             /* a pupil candidate was accepted */
} iris_quality;

typedef struct iris_match_result {
    float score;                   /* normalized fractional Hamming distance (lower = better) */
    float raw_hd;                  /* raw fractional Hamming distance at the best shift */
    uint32_t bits_compared;        /* commonly valid bits at the best shift */
    int32_t shift;                 /* best rotation, in code columns */
    int32_t probe_index;           /* best code inside each template */
    int32_t gallery_index;
} iris_match_result;

typedef struct iris_enroll_info {
    uint32_t samples_in;
    uint32_t samples_used;         /* consistent samples (score to medoid <= enroll_consistency) */
    uint32_t codes_stored;
    float mean_intra_score;        /* mean pairwise score among used samples */
    float max_intra_score;
} iris_enroll_info;

typedef struct iris_engine iris_engine;
typedef struct iris_template iris_template;
/* Per-thread working memory of iris_extract_ex (since API 2). Not thread safe: one per thread. */
typedef struct iris_context iris_context;

/* Expected position of the iris (input image coordinates), e.g. iris_x/iris_y/iris_r of the
 * previous frame. Since API 2. */
typedef struct iris_search_hint {
    float x, y;  /* iris centre */
    float r;     /* iris radius; <= 0 disables the hint */
} iris_search_hint;

IRIS_API int iris_engine_api_version(void);
IRIS_API const char *iris_status_string(iris_status status);

IRIS_API void iris_config_default(iris_config *config);
/* config may be NULL for defaults. Returns IRIS_E_INVALID_ARGUMENT on an inconsistent config. */
IRIS_API iris_status iris_engine_create(const iris_config *config, iris_engine **engine_out);
IRIS_API void iris_engine_destroy(iris_engine *engine);
IRIS_API void iris_engine_get_config(const iris_engine *engine, iris_config *config_out);

/*
 * Extract a one-code template from an image. On success *template_out is a new template;
 * on failure it is set to NULL. quality may be NULL.
 */
IRIS_API iris_status iris_extract_image(const iris_engine *engine, const iris_image *image,
                                        iris_template **template_out, iris_quality *quality);
/* Convenience: 8-bit grey image, stride in bytes (0 = width). */
IRIS_API iris_status iris_extract(const iris_engine *engine, const uint8_t *gray, int32_t width,
                                  int32_t height, int32_t stride, iris_template **template_out,
                                  iris_quality *quality);

/*
 * Since API 2. Same result as iris_extract_image(), faster on video:
 * - `context` (optional) keeps the working buffers between calls, so steady-state calls do not
 *   allocate; create one per thread with iris_context_create().
 * - `hint` (optional) restricts the eye search to a window around the given iris position first
 *   (about 2.2 radii around it); when no eye is found there, the whole image is searched as
 *   usual. The segmentation, template and quality are those of the eye found.
 */
IRIS_API iris_status iris_context_create(const iris_engine *engine, iris_context **context_out);
IRIS_API void iris_context_destroy(iris_context *context);
IRIS_API iris_status iris_extract_ex(const iris_engine *engine, iris_context *context,
                                     const iris_image *image, const iris_search_hint *hint,
                                     iris_template **template_out, iris_quality *quality);

/* Compare two templates (minimum over all code pairs and rotations). */
IRIS_API iris_status iris_match(const iris_engine *engine, const iris_template *probe,
                                const iris_template *gallery, float *score_out);
IRIS_API iris_status iris_match_ex(const iris_engine *engine, const iris_template *probe,
                                   const iris_template *gallery, iris_match_result *result_out);
/* *accepted_out = score <= config.match_threshold. score_out may be NULL. */
IRIS_API iris_status iris_verify(const iris_engine *engine, const iris_template *probe,
                                 const iris_template *gallery, int32_t *accepted_out,
                                 float *score_out);

/*
 * Build an enrollment template from n samples (each usually from iris_extract). Inconsistent
 * samples are discarded; up to enroll_max_codes codes are kept. info may be NULL.
 */
IRIS_API iris_status iris_enroll(const iris_engine *engine, const iris_template *const *samples,
                                 size_t n_samples, iris_template **template_out,
                                 iris_enroll_info *info);

IRIS_API size_t iris_template_code_count(const iris_template *tmpl);
IRIS_API iris_template *iris_template_clone(const iris_template *tmpl);
IRIS_API void iris_template_destroy(iris_template *tmpl);

/*
 * Versioned little-endian serialization with CRC-32. Call with buffer == NULL (or too small) to
 * obtain the size in *size_out (IRIS_E_BUFFER_TOO_SMALL when buffer is non-NULL but small).
 */
IRIS_API iris_status iris_template_serialize(const iris_template *tmpl, uint8_t *buffer,
                                             size_t capacity, size_t *size_out);
IRIS_API iris_status iris_template_deserialize(const uint8_t *buffer, size_t size,
                                               iris_template **template_out);

/*
 * Theoretical false accept rate of one comparison at a Hamming-distance threshold, for an
 * impostor distribution modelled as binomial with `degrees_of_freedom` (Daugman). The bench tool
 * estimates the degrees of freedom from measured impostor scores.
 */
IRIS_API double iris_binomial_far(double threshold, double degrees_of_freedom);

#ifdef __cplusplus
}
#endif

#endif /* CITYMAN_IRIS_ENGINE_H */
