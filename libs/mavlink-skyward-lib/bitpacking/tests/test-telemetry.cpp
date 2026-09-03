#define CATCH_CONFIG_MAIN
#include "include/catch.hpp"

//#include <stdio.h>
//#include <unistd.h>
#include "hermes/HermesPackets.h"

uint8_t buf[HighRateTMPacker::HR_TM_PACKET_SIZE];

TEST_CASE("Test HR_TELEMETRY")
{
    GIVEN("A HR telemetry packer") {
        HighRateTMPacker packer{buf};
        long long ts;

        WHEN("writing multiple values to the same field") {
            packer.packTimestamp(1111111, 0);
            packer.packTimestamp(123, 1);
            packer.packTimestamp(255, 1);
            packer.packTimestamp(1111111, 2);
            packer.packTimestamp(1, 2);
            packer.packTimestamp(12345678, 3);

            THEN("the last written value is read") {
                REQUIRE(packer.unpackTimestamp(&ts, 0));
                REQUIRE(ts == 1111111);

                REQUIRE(packer.unpackTimestamp(&ts, 1));
                REQUIRE(ts == 255);

                REQUIRE(packer.unpackTimestamp(&ts, 2));
                REQUIRE(ts == 1);

                REQUIRE(packer.unpackTimestamp(&ts, 3));
                REQUIRE(ts == 12345678);
            }
        }

        WHEN("writing an ivnalid position") {
            THEN("the operation fails") {
                REQUIRE_FALSE(packer.packTimestamp(123, 4));
                REQUIRE_FALSE(packer.packTimestamp(123, -1)); // CHE SUCCEDE SE PASSO -1 ??
            }
        }

        WHEN("filling an entire packet multiple times") {
            // write 4 times the 4 subpackets
            for(int i = 1; i < 5; i++) {
                for(int j = 0; j < 4; j++) {
                    packer.packTimestamp(123+i*2, j);

                    packer.packPressureAda(55000.79853+i*20, j);
                    packer.packPressureDigi(55000.79853+i*20, j);

                    packer.packMslAltitude(1234.5678+i*2, j);
                    packer.packAglAltitude(1234.5678+i*2, j);

                    packer.packVertSpeed(123.456+i*2, j);
                    packer.packVertSpeed2(123.456+i*2, j);

                    packer.packAccX(123.456+i*2, j);
                    packer.packAccY(-123.456+i*2, j);
                    packer.packAccZ(123.456+i*2, j);

                    packer.packGyroX(5.4+i*2, j);
                    packer.packGyroY(5.4+i*2, j);
                    packer.packGyroZ(5.4+i*2, j);

                    packer.packGpsLat(41.822766, j);
                    packer.packGpsLon(14.034714, j);
                    packer.packGpsAlt(123.456+i*2, j);

                    packer.packFmmState(1+i, j);
                    packer.packDplState(1+i, j);
                    if (i % 2 == 0) {
                        packer.packPinLaunch(0, j);
                        packer.packPinNosecone(0, j);
                        packer.packGpsFix(0, j);
                    }
                    else {
                        packer.packPinLaunch(1, j);
                        packer.packPinNosecone(1, j);
                        packer.packGpsFix(1, j);
                    }
                }
            }
            THEN("only the last written values are read") {
                long long ts;
                float f;
                uint8_t u;
                double d;

                for (int j = 0; j < 4; j++) {
                    REQUIRE(packer.unpackTimestamp(&ts, j));
                    REQUIRE(ts == 131);

                    REQUIRE(packer.unpackPressureAda(&f, j));
                    REQUIRE(Approx(f).margin(13.42773438) == 55080.79853);
                    REQUIRE(packer.unpackPressureDigi(&f, j));

                    REQUIRE(Approx(f).margin(6.713867188) == 55080.79853);
                    REQUIRE(packer.unpackMslAltitude(&f, j));
                    REQUIRE(Approx(f).margin(1) == 1234.5678+8);
                    REQUIRE(packer.unpackAglAltitude(&f, j));
                    REQUIRE(Approx(f).margin(6.0546875) == 1234.5678+8);

                    REQUIRE(packer.unpackVertSpeed(&f, j));
                    REQUIRE(Approx(f).margin(0.5859375) == 123.456+8);
                    REQUIRE(packer.unpackVertSpeed2(&f, j));
                    REQUIRE(Approx(f).margin(0.5) == 123.456+8);

                    REQUIRE(packer.unpackAccX(&f, j));
                    REQUIRE(Approx(f).margin(0.1533203125) == 123.456+8);
                    REQUIRE(packer.unpackAccY(&f, j));
                    REQUIRE(Approx(f).margin(0.1533203125) == -123.456+8);
                    REQUIRE(packer.unpackAccZ(&f, j));
                    REQUIRE(Approx(f).margin(0.1533203125) == 123.456+8);

                    REQUIRE(packer.unpackGyroX(&f, j));
                    REQUIRE(Approx(f).margin(1.953125) == 5.4+8);
                    REQUIRE(packer.unpackGyroY(&f, j));
                    REQUIRE(Approx(f).margin(1.953125) == 5.4+8);
                    REQUIRE(packer.unpackGyroZ(&f, j));
                    REQUIRE(Approx(f).margin(1.953125) == 5.4+8);

                    REQUIRE(packer.unpackGpsLat(&d, j));
                    REQUIRE(Approx(d).margin(0.00002799658203) == 41.822766);
                    REQUIRE(packer.unpackGpsLon(&d, j));
                    REQUIRE(Approx(d).margin(0.00002799658203) == 14.034714);

                    REQUIRE(packer.unpackGpsAlt(&f, j));
                    REQUIRE(Approx(f).margin(5) == 123.456+8);

                    REQUIRE(packer.unpackFmmState(&u, j));
                    REQUIRE(u == 5);
                    REQUIRE(packer.unpackDplState(&u, j));
                    REQUIRE(u == 5);
                    REQUIRE(packer.unpackPinLaunch(&u, j));
                    REQUIRE(u == 0);
                    REQUIRE(packer.unpackPinNosecone(&u, j));
                    REQUIRE(u == 0);
                    REQUIRE(packer.unpackGpsFix(&u, j));
                    REQUIRE(u == 0);
                }
            }
        }
        WHEN("writing timestamps in a loop") {
            long long ts;
            for(int i = 0; i < 100; i++) {
                REQUIRE(packer.packTimestamp(123+i, 0));
                REQUIRE(packer.unpackTimestamp(&ts, 0));
                REQUIRE(ts == 123+i);
                //printf("timestamp = %ld\n", ts);
                //usleep(200000);
            }
        }
    }
}

TEST_CASE("Test LR_TELEMETRY")
{
    GIVEN("A LR telemetry packer") {
        LowRateTMPacker packer{buf};
        long long ts;

        WHEN("writing multiple values to the same field") {
            packer.packLiftoffTs(1111111, 0);
            packer.packLiftoffTs(12345678, 0);
            packer.packLiftoffTs(123, 0);

            THEN("the last written value is read") {
                REQUIRE(packer.unpackLiftoffTs(&ts, 0));
                REQUIRE(ts == 123);
            }
        }

        WHEN("writing an ivnalid position") {
            THEN("the operation fails") {
                REQUIRE_FALSE(packer.packLiftoffTs(123, 4));
                REQUIRE_FALSE(packer.packLiftoffTs(123, 1));
                REQUIRE_FALSE(packer.packLiftoffTs(123, -1));
            }
        }
    }
}