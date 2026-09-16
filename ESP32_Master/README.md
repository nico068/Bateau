# Bateau — ESP32 maître

Console de commande ESP-IDF : joystick, encodeurs incrémentaux, BME280, MPU9250 et CAN interne vers l'ESP32 des actionneurs.

## Démarrage

```bash
idf.py set-target esp32
idf.py menuconfig
idf.py build flash monitor
```

Le CAN interne fonctionne à 500 kbit/s sur GPIO17/TX et GPIO16/RX. Les commandes utilisent une valeur servo normalisée de 0 à 1000. Le pilote CAN et le protocole bateau sont séparés. La configuration matérielle se trouve dans `components/board_config/include/app_config.h`.

Les trois encodeurs utilisent PCNT en quadrature. Les GPIO 34 et 35 n'ont pas de résistances de tirage internes : l'encodeur du volant exige des résistances externes.

Les manettes démarrent à zéro. Le volant démarre au centre. Maintenir le bouton du joystick sélectionne temporairement le joystick X à la place du volant pour orienter les turbines.

Le BME280 est lu à 2 Hz. Le MPU9250 est lu à 50 Hz. Chaque capteur possède son propre composant et peut être absent sans empêcher la commande CAN de démarrer. Les encodeurs, le gyroscope et le magnétomètre doivent être calibrés sur le montage réel.
