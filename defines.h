

// Macros for specifying type alignment
#if (defined __GNUC__) || (defined __INTEL_COMPILER)
  #define CHRONO_ALIGN_16 __attribute__ ((aligned(16)))
#elif defined _MSC_VER
  #define CHRONO_ALIGN_16 __declspec(align(16))
#else
  #define CHRONO_ALIGN_16
#endif


#if defined _MSC_VER
#define fmax max
#define fmin min
#endif

#define CHRONO_PARALLEL_USE_DOUBLE
