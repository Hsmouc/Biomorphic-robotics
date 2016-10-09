/****************************************
 * 文件名       ：assert.h
 * 描述         ：断言宏定义头文件

**********************************************************************************/


#ifndef _ASSERT_H_
#define _ASSERT_H_

/********************************************************************/

void assert_failed(char *, int);

#ifdef DEBUG
#define ASSERT(expr) \
    if (!(expr)) \
        assert_failed(__FILE__, __LINE__)
#else
#define ASSERT(expr)
#endif

/********************************************************************/
#endif /* _ASSERT_H_ */

