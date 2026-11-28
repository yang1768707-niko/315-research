#ifndef COMM_TEST_PROTOCOL_H
#define COMM_TEST_PROTOCOL_H
/* Identical wire definitions in node, coordinator and gateway projects. */
#define COMM_TEST_VERSION 1U
#define COMM_TEST_REQUEST 0xB0U
#define COMM_TEST_REPLY 0x17U
#define COMM_TEST_REPORT 0xA6U
#define COMM_TEST_REQUEST_LEN 5U
#define COMM_TEST_REPLY_LEN 7U
#define COMM_TEST_REPORT_LEN 10U
#define COMM_TEST_TIMEOUT_MS 10000U
static inline unsigned long comm_test_token(const unsigned char *p)
{
    return (unsigned long)p[1]|((unsigned long)p[2]<<8)|
        ((unsigned long)p[3]<<16)|((unsigned long)p[4]<<24);
}
static inline void comm_test_request(unsigned char *p,unsigned long token)
{
    p[0]=COMM_TEST_VERSION;
    for(unsigned i=0;i<4;i++)p[i+1]=(unsigned char)(token>>(8*i));
}
#endif
