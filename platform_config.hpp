// (Platform Configuration)
#define UU_PLATFORM_CONFIG
#define CPU_IA32 (1)   // @doc{32bit intel platform}
#define CPU_X86_64 (2) // @doc{x64 introduced by AMD and adopted by intel}
#define OS_OSX (1)
#define OS_WINDOWS (2)
#define COMPILER_CLANG (1)
#define COMPILER_MSC (2)
#if OS == OS_OSX
#if !defined(CPU)
#define CPU CPU_X86_64
#endif
#endif
#if !defined(COMPILER) && defined(__clang__)
#define COMPILER COMPILER_CLANG
#endif
#if !defined(COMPILER) && defined(_MSC_VER)
#define COMPILER COMPILER_MSC
#endif
#if !defined(OS)
#error "OS must be defined"
#endif
#if !defined(CPU)
#error "CPU must be defined"
#endif
#if COMPILER == COMPILER_CLANG
#define CLANG_ATTRIBUTE(x) __attribute__((x))
#define uu_debugger_break() DOC("invoke debugger") asm("int3")
#else
#define CLANG_ATTRIBUTE(x)
#define uu_debugger_break() DOC("invoke debugger") __debugbreak()
#endif
