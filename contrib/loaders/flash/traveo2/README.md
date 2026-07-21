## Implementation of CY_ASSERT macro in cy_syslib.h file is buggy!
In Release mode (when NDEBUG is defined) CY_ASSERT(x) macro is defined 
as follows:
```c
    #define CY_ASSERT(x)    do  \
                            {   \
                            }   \
                            while(0u)
```
This macro does not evaluate expression `X` thus breaks some invocation schemes when `X` contains function call:
```c
CY_ASSERT(SomeFunctionCall() == CY_SUCCESS);
```
effectively converting them to no-op. Currently the following definition is used:
```c
#define CY_ASSERT(x)   if(!(x)) while(1u);
```
## Diff / patch process
diff -ruN SDL_orig/ sdl/ > sdl_cumulative.patch
patch --binary -p0 < sdl_cumulative.patch
