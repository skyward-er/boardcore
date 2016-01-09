#ifndef BOARD_H
#define BOARD_H


class Board : public Singleton<Board> {
    friend class Singleton<Board>;
public:
    CanManager *getCanManager() const = 0;
private:
    Board() = 0;
};

class Stm32TestBoard : public Board {
    friend class Singleton<Stm32TestBoard>;
public:
    CanManager *getCanManager() const {
        return mCanManager; 
    }

private:
    Board() {
        mCanManager.addBus( {
        //   ADDR | GPIO BUS | RX | TX |      MODE      | ALT
            {CAN1, GPIOA_BASE, 11,  12, Mode::ALTERNATE,  9},
            {CAN2, GPIOB_BASE,  5,   6, Mode::ALTERNATE,  9},
        );
    }

    CanManager mCanManager;
}

#endif /* BOARD_H */
