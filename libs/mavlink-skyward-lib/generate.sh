cd  mavlink
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=1.0 --output=../mavlink_lib ../message_definitions/orion.xml
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=1.0 --output=../mavlink_lib ../message_definitions/orion.xml