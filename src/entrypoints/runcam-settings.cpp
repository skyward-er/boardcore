#include "miosix.h"

#include <drivers/runcam/Runcam.h>



int main(){


    printf("Press d to move down, c to confirm the action and o to open the menu");
    Runcam test;
    
    if(test.init()){
        char c;
       while(true){
            scanf("%c",&c);

            if(c=='d')
            {
                printf("d pressed");
                //test.moveDown();
            }else if(c=='c'){
                printf("c pressed");
                //test.selectSetting();
            }else if(c=='o'){
                printf("o pressed");
                //test.openMenu();
            }
        }


        return 0;
    }

    return -1;
}