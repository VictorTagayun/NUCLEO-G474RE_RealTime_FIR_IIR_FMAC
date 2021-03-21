/* Filter parameter P: number of feed-forward taps or coefficients in the range [2:127] */
#define COEFF_VECTOR_B_SIZE     5

/* Filter parameter Q: not used */
#define FILTER_PARAM_Q_NOT_USED 0

/* Filter parameter R: gain in the range [0:7] */
#define GAIN                    7 // gain = 1


/* Throughput parameter: extra space in the input buffer (minimum: 0) */
#define MEMORY_PARAMETER_D1     1

/* Throughput parameter: extra space in the output buffer (minimum: 1) */
#define MEMORY_PARAMETER_D2     2

/* Throughput parameter: watermark threshold for the input buffer */
#define INPUT_THRESHOLD         FMAC_THRESHOLD_1

/* Throughput parameter: watermark threshold for the output buffer (inferior or equal to MEMORY_PARAMETER_D1) */
#define OUTPUT_THRESHOLD        FMAC_THRESHOLD_1


/* FMAC internal memory configuration: base address of the coefficient buffer */
#define COEFFICIENT_BUFFER_BASE 0

/* FMAC internal memory configuration: size of the coefficient buffer */
#define COEFFICIENT_BUFFER_SIZE COEFF_VECTOR_B_SIZE

/* FMAC internal memory configuration: base address of the input buffer */
#define INPUT_BUFFER_BASE       COEFFICIENT_BUFFER_SIZE

/* FMAC internal memory configuration: size of the input buffer */
#define INPUT_BUFFER_SIZE       COEFF_VECTOR_B_SIZE + MEMORY_PARAMETER_D1

/* FMAC internal memory configuration: base address of the input buffer */
#define OUTPUT_BUFFER_BASE      COEFFICIENT_BUFFER_SIZE + INPUT_BUFFER_SIZE

/* FMAC internal memory configuration: size of the input buffer */
#define OUTPUT_BUFFER_SIZE      MEMORY_PARAMETER_D2

