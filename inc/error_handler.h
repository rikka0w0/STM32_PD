#ifndef __ERRHANDLER_H__
#define __ERRHANDLER_H__

// #define USE_FULL_ASSERT 1U
#define USE_ERROR_HANDLER 1U

#ifdef __cplusplus
 extern "C" {
#endif

void _Error_Handler(char *, int);

#ifdef USE_ERROR_HANDLER
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#else
#define Error_Handler()
#endif
#ifdef __cplusplus
}
#endif

#endif /* __ERRHANDLER_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
