/* tools/astindex shim. DirectXMath.h includes <cpuid.h> when __clang__ is defined; the libclang
   wheel ships no clang resource headers, so without this the include is a FATAL error and the
   rest of every TU that reaches DirectXMath is never parsed. Only the index sees this file. */
#pragma once
