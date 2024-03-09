# Skyward Registry

### Goal and purpose

Skyward Registry is a Skyward project to develop a configuration saving
mechanism into persistent (flash memory) and non persistent memories (RAM).

Its purpose is to save the configurations for the rocket and retrieve them after
a reboot or momentary CPU issues and transient events.

The need of such sw development has its roots in some events in which
the reboot of the rocket, and therefore its configuration, lent to a misconfiguration, taking defaults values. This caused the propulsion chamber's valves
to open with wrong timing and therefore causing a misfire.
Therefore, it is of the utmost importance to save the configuration and avoid default and hardcoded values.

## The front-end

### Invocation examples
In this case we take as example a ficticius configuration entry (NAME which has as value datatype a float)

Type-unsafe interface methods (the one for now tested and considered as most used):

#### setConfigurationUnsafe
```cpp
float value = 1.3;
/*! The id could also be a specific enum (will be casted to uint32_t)*/
uint32_t id = 10;

if(frontEnd.setUnsafe(id, value))
 { /*! correctly set */
}

```
#### getConfigurationUnsafe
```cpp
float value;
/*! The id could also be a specific enum (will be casted to uint32_t)*/
uint32_t id;


if(frontEnd.getConfigurationUnsafe(id, value)) {
    /*! Getted the value */
    }
```

#### getConfigurationOrDefaultUnsafe
```cpp
uint32_t ignitionTime, ignitionDefault = 200;
/*! The id could also be a specific enum (will be casted to uint32_t)*/
uint32_t id = 0;
/*! Default value that will be get (and possibly set) if cannot get an already initialized value*/
uint32_t default = 400;

ignitionTime = frontEnd.getConfigurationOrDefaultUnsafe(id, ignitionDefault);
```

Type-safe interface methods:

#### setConfiguration
```cpp
/*! Structure from the OBSW structures*/
Ignition ignitionTime(2.0);
frontEnd.setConfiguration(ignitionTime);
```
#### getConfiguration
```cpp
/*! Structure from the OBSW structures*/
Ignition ignitionTime;
if(!frontEnd.getConfiguration(ignitionTime)){
    /*! Error getting the configuration value */
}
```

#### getConfigurationOrDefault
```cpp
/*! Structure from the OBSW structures*/
Ignition ignitionTime, ignitionDefault(2.0);
ignitionTime = frontEnd.getConfigurationOrDefault(ignitionDefault);
```

#### arm
```cpp
frontEnd.arm()
```

#### disarm
```cpp
frontEnd.disarm()
```

#### isConfigurationEmpty
```cpp
if(frontEnd.isConfigurationEmpty())
{
    /*! The front end configuration is empty */
}
```

#### save
```cpp
frontEnd.save();
```

#### load
```cpp
frontEnd.load();
```

#### clear
```cpp
frontEnd.clear();
```

### How to add new structs
The correct flow to add new types/configuration entries is:

- `TypeStructure.h`: If not exist, create a new struct for the type that we will use for the configuration entry value. The structures in OBSW will refer to these structures. 

- `RegistryFrontend.h`: Remember to update:
The type enum; 
The TypeUnion fields;
Add in EntryStructsUnion the getFromUnion, setUnion and getFromSerializedVector overloads;
Modify the and appendSerializedFromUnion for the serialization also of such new type.
 


### Goals

| Goal |  Description  |
|:-----|:--------:|
| G1 | The configuration will be saved in memory |
|G2 | It will be possible to set the value of a particular configuration entry|
|G3 | It will be possible to get the value of a particular configuration entry |
|G4 | It will be possible to inspect the configured entries |
|G5 | More data types can be used as value for the different configurations entries |
|G6 | There will be a protection against changes to the configuration during flight |
|G7 | No dynamic allocations will be executed during the flight |
|G8 | The registry will offer a persistent configuration saving |
|G9 | It will be possible to verify the integrity of the configuration|
|G10 | It will be possible to explore the current configuration |
|G11 | Thread safeness is guaranteed |
|G12| Some methods also guarantees type safeness|

### Assumptions
The front-end, FE, considers some important assumptions about the usage of such sw component.
This assumptions are about the method call, the order of operations and other assumptions on the usage.
Such assumptions considers the actual necessities for Skyward Registry and might not be true in future.

| Assumption |  Description  |
|:-----|:--------:|
|A1 | The FE is constructed and instantiated once, a single time|
|A2 | The FE does saves and retrieves a single configuration from the registry and no multiple versions|
|A3 | The FE is called before the flight phase for instantiation and registry check |
|A4 | The FE is correctly armed before the flight |
|A5 | The FE disarming is not used during flight phase |
|A6 | The caller considers the return of the get and set methods|
|A7 | Configuration entries does not have pointers as datatype |
|A8| The backend does not modify the vector to be saved.|
|A9| Other methods not modify the vector through getSerializedConfiguration|

### Requirements

| Requirements |  Description  |
|:-----|:--------:|
|R1 | The interface must allow setting a value for the specific configuration entries |
|R2 | The interface must allow getting a specified value for a   configuration entries |
|R3 | The interface must perform some type check for the   specifics entries to get |
|R4 | The interface must not allocate memory during the  flight phase |
|R5 | The interface must not change the configuration entries during flight phase |
|R6 | The interface does save the configuration entries using the back-end components |
|R7 | The FE must manage concurrent get/set by multiple threads (Thread safety)|
|R8 | The FE must manage concurrent arm/disarm by multiple threads  (Thread safety)|
|R9 | The FE must allow exploration of the actual configuration|
|R10 | The FE should be able to control the configuration state (empty or has elements)|

### Interface methods

The unsafe (type-unsafe) methods does not use the proper data structure for the set and get but instead just pass a parameter value for a specific registry entry uint32_t index identifier (could be a casted enumerator from OBSW structures)

- `setConfiguration[Unsafe]`:  A setter method is in need to ensure R1. This method should allow 
    insert a value for a specific configuration entry while guarantee the different data types for the values (R3).
    
- `getConfiguration[Unsafe]`:  A getter method is in need to ensure the visibility of the configuration. 
    Such method should get a value for a specified configuration entry and changes the value to passed by reference value parameter. It does check that the type is consistent with the saved one.

- `getOrSetDefaultConfiguration[Unsafe]`: A particular get method which returns the get value and if it is not existing in the configuration set it (if possible!) and returns the default value.

 - `arm`:  An "arm" method which guarantees no memory allocations and no changes to the configuration are in place
    until a "disarm" method. It is an important functionality to ensure a "safe" memory configuration during flight.

 - `disarm`: A "disarm" method, to disable the stable and "safe" configuration mode of the registry to allow again 
    allocations and settings on the configuration.

 - `isConfigurationEmpty`: A method to know if there is an existing configuration or if it is empty

- `configuredEntries`: A method which returns the already existing entries of the configuration as a set.

- `load`: Loads from the backend the configuration. If no backend, it loads it from its own vector. 

- `save`: Saves the configuration to the backend.

- `clear`: Clears the configuration both in frontend and backend components, starting with an empty configuration.

- `visitConfiguration`: Given a callback, it does apply it for each element of the actual configuration with the id and EntryStructUnion parameters. 

### Data structures
The data structures are managed in 2 main header files.
#### TypeStructures.h
Type structures have:

- `RootTypeStructure`: A root type, with just 2 template attributes: value and index

- `(Float|UInt32|...)Type`: A sub-type that does specify the value attribute type. It inherits from RootTypeStructure

#### RegistryFrontend.h

- `TypeUnion`: The union type for saving the values of the different configuration entries

- `EntryStructsUnion`: The structure actually saved into the configuration, with the value and type attributes. Also, it does expose all the useful operations for get/set the value from/to union, append to the serialize vector a value, get a value from the serialized vector.

#### Data saving serialization
serializationVector is a vector that after getSerializedVector will contain:
|||||||||
|:-----|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|----:|
|8B zero| Nr. entries | Length vector | 4B CRC-Checksum | ID_0 | TypeID_0 | Value_0| ... |

The vector will contain only the set configurations. After the header, there will be all the configured entries with configuration ID, Type ID, Value(s).

In case of multiple values, they are one after the other 

e.g. for a Coordinate: 
|||||
|:-----|:--------:|:--------:|--------:|
| ID: 2 | TypeID: 1 | Val_1: 45.50109 | Val_2: 9.15633 | 

Where TypeID in this example is a coordinates type and therefore 45.50109 is the latitude and 9.15633 the longitude

### Saving
The save of the configuration is done at each new entry configured, each set will held to a save of the configuration, mediated by the middleware which might lead to discard late writes.

## The middleware
Another part of the Registry is the middleware which decouples the registry front-end and the backend. This aims to avoid the block given by waiting the SD or underlying saving backend for write the serialized configuration.

This component avoid this by using a buffer for the write to backend and another, at disposal for writes from the front-end. It is an active object which waits for new data to write it to backend.

### Methods
- `write`: Writes to the backend the given configuration. The real write is done by the run() method executed by the
registry middleware's thread, while in reality write just writes to the buffer the serialized configuration that will
be then written by the thread.
- `load`: Loads into the vector the saved configuration from the backend. Returns false if none is saved.
- `clear`: Clears/deletes the configuration in buffers and underlying backend