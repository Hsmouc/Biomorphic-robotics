/******************** ********************
 * �ļ���       ��assert.c
 * ����         ������������������ִ�к���
 *

**********************************************************************************/

/*
 * File:        assert.c
 * Purpose:     Provide macro for software assertions
 *
 * Notes:       ASSERT macro defined in assert.h calls assert_failed()
 */

#include "common.h"

//const char ASSERT_FAILED_STR[] = "Assertion failed in %s at line %d\n";
const char ASSERT_FAILED_STR[] = "���Է�����%s�ļ��ĵ�%d�У�\n�����������������������ԭ��\t���� by**\n\n";

/********************************************************************/
//������������������������˴���״̬���ͻ��ӡ������Ϣ����LED����ʾ״̬
void assert_failed(char *file, int line)
{
    //int i;
    LED_init();
    while (1)
    {
#ifdef DEBUG_PRINT
        printf(ASSERT_FAILED_STR, file, line);  //��ӡ������Ϣ
#endif
        //for (i = 0xffffff; i; i--) ;
        water_lights();                         //����ˮ����ָʾ�������״̬
        //for (i = 0xffffff; i; i--) ;
    }
}
/********************************************************************/