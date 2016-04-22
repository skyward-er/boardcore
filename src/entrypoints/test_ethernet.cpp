
#include <Common.h>
#include <ethernet/UdpSocket.h>

using namespace miosix;
using namespace std;

int main()
{
    uint8_t ipAddr[] = {192,168,1,30};                        //Device's IP address
    uint8_t mask[] = {255,255,255,0};                         //Subnet mask
    uint8_t destIp[] = {192,168,1,4};                        //Destination IP address
    
    unsigned char message[] = "message from the hell...\n";
    
    W5200::instance().setIpAddress(ipAddr);
    W5200::instance().setSubnetMask(mask);
    
    UdpSocket sock(2020);
    
    uint8_t buf[50];
    uint8_t sip[4];
    uint16_t port, len = 0;
    
    while(1)        
    {
       len = sock.receive(sip, &port, buf);
       printf("received %d bytes from %d.%d.%d.%d:%d\n",len,buf[0],buf[1],buf[2],buf[3],port);
       
//         uint16_t len = static_cast<uint16_t>(sizeof(message)/sizeof(unsigned char));
//         sock.sendTo(destIp,4040,message,len);
        Thread::sleep(500);
    }
    
    return 0;
}


// #include <cstdio>
// #include <kernel/scheduler/scheduler.h>
// #include <Common.h>
// #include <ethernet/W5200/w5200.h>
// // #include "miosix/miosix.h"
// // #include "w5200.h"
// 
// using namespace std;
// using namespace miosix;
// 
// 
// uint8_t irreg = 0;
// int icount = 0;
// W5200& ethPhy = W5200::instance();
// 
// // Thread *waiting = 0;
// 
// // void __attribute__((naked))  EXTI1_IRQHandler()
// // {
// //     saveContext();
// //     asm volatile("bl _Z13EXTIrqHandlerv");
// //     restoreContext();
// // }
// // 
// // void __attribute__((used))  EXTIrqHandler()
// // {
// //     EXTI->PR |= EXTI_PR_PR1;
// //     irreg = ethPhy.getSocketInterruptReg(0);
// //     ethPhy.clearSocketInterruptReg(0);
// //     
// //     bool wakeup = false;
// //      
// //     if(irreg & 0x04){
// // //         ledOn();
// //         wakeup = true;
// //     }
// //     
// //     if(waiting==0 || wakeup == false) return;
// //     waiting->IRQwakeup();
// //     if(waiting->IRQgetPriority()>Thread::IRQgetCurrentThread()->IRQgetPriority())
// //         Scheduler::IRQfindNextThread();
// //     waiting=0;
// // }
// 
// int main()
// {       
//     eth::int1::mode(Mode::INPUT);    
//     
//     {
//         FastInterruptDisableLock dLock;
//         RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//         RCC_SYNC();
//     }
//     
//     SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;
//     EXTI->IMR |= EXTI_IMR_MR1;
//     EXTI->FTSR |= EXTI_FTSR_TR1;
//     
//     NVIC_SetPriority(EXTI1_IRQn,10);
//     NVIC_ClearPendingIRQ(EXTI1_IRQn);
//     NVIC_EnableIRQ(EXTI1_IRQn);
//     
//     uint8_t ipAddr[] = {192,168,1,30};                        //Device's IP address
//     uint8_t mask[] = {255,255,255,0};                         //Subnet mask
//     uint8_t destIp[] = {192,168,1,4};                        //Destination IP address
//     
//     unsigned char message[] = "message from the hell...\n";
//     
//     SOCKET sockn = 0;           
//     ethPhy.setIpAddress(ipAddr);
//     ethPhy.setSubnetMask(mask);
//     
//     ethPhy.setModeReg(0x00);
//     ethPhy.setSocketInterruptMask(0xFF);
//     
//     ethPhy.setSocketModeReg(sockn,SOCKn_MR_UDP);   //set socket 0 in UDP mode
//     ethPhy.setSocketInterruptMaskReg(sockn,0xFF);
// 
//     ethPhy.setSocketSourcePort(sockn,2020);        //set socket 0 source port
//     ethPhy.setSocketDestIp(sockn,destIp);          //set socket 0 destination IP adddress
//     ethPhy.setSocketDestPort(sockn,4040);          //set socket 0 destination port
//            
//     ethPhy.setSocketCommandReg(sockn,SOCKn_CR_OPEN);   //open socket 0        
// 
//     uint8_t buf[100];
//     
//     while(1)
//     {                
//         ethPhy.writeData(sockn,message,25);
//         ethPhy.setSocketCommandReg(sockn,SOCKn_CR_SEND);
// //         ethPhy.setSocketCommandReg(sockn,SOCKn_CR_RECV);                                     
// //         
// //         waiting = Thread::getCurrentThread();
// //         {
// //             FastInterruptDisableLock dLock;
// //             while(waiting)
// //             {                
// //                 waiting->IRQwait();
// //                 {
// //                     FastInterruptEnableLock eLock(dLock);                    
// //                     Thread::yield();
// //                 }
// //             }
// //         }
// //         
// //         uint16_t len = ethPhy.getReceivedSize(sockn);        
// //         ethPhy.readData(sockn,buf,len);
// //         
// //         iprintf("received from %d.%d.%d.%d:%d\n",buf[0],buf[1],buf[2],buf[3],(buf[4]<<8) | buf[5]);
//                 
//         Thread::sleep(200);
//         ledOff();
//         Thread::sleep(300);
//     }
// }
