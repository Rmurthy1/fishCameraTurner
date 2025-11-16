#ifndef NETWORKING_H
#define NETWORKING_H

#include <Arduino.h>


class Networking {
public:
    void readDataFromFirebase(String lastData, String endpoint,  bool &success);
};

#endif // NETWORKING_H