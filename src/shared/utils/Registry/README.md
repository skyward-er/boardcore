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
In this case we take as example a fictitious configuration entry with uint32_t configuration `id` and value `value`

Type-unsafe interface methods (which takes any data type given as value, without having any map id->data type if the entry was not already set):

#### setConfigurationUnsafe
```cpp
float value = 1.3;
// The id could also be a specific enum (will be casted to uint32_t)
uint32_t id = 10;

if(frontEnd.setUnsafe(id, value) == RegistryError::OK)
{    
    // correctly set
}

```
#### getConfigurationUnsafe
```cpp
float value;
// The id could also be a specific enum (will be casted to uint32_t)
uint32_t id;


if(frontEnd.getConfigurationUnsafe(id, value) == RegistryError::OK) 
{
    /* Getted the value */
}
```

#### getConfigurationOrDefaultUnsafe
```cpp
uint32_t ignitionTime, ignitionDefault = 200;
//The id could also be a specific enum (will be casted to uint32_t)
uint32_t id = 0;
/* Default value that will be get (and possibly set) if cannot get an already initialized value*/
uint32_t default = 400;

ignitionTime = frontEnd.getConfigurationOrDefaultUnsafe(id, ignitionDefault);
```

#### arm
```cpp
frontEnd.arm()
```

#### disarm
```cpp
frontEnd.disarm()
```

#### isEmpty
```cpp
if(frontEnd.isEmpty())
{
    /* The front end configuration is empty */
}
```

#### save
```cpp
if(frontEnd.save() == RegistryError::OK)
{
    // Saved correctly the configuration
}
```

#### load
```cpp
if(frontEnd.load() == RegistryError::OK){
    // Loaded the configuration correctly
}
```

#### clear
```cpp
frontEnd.clear();
```

### How to add new structs
The correct flow to add new types/configuration entries is:

If the data type needed not exists already you need to follow these steps:

- `TypeStructure.h`: 
    - Add the type to the TypeUnion;
    - Update the type enum; 
    - Create the correct overloads in EntryStructsUnion;

 - `RegistrySerializer.h`, `RegistrySerializer.cpp`: 
    - Update the write
    - Update the deserialize


### Goals

| Goal |  Description  |
|:-----|:--------:|
| G1 | The configuration could be possibly saved in memory |
| G2 | The configuration could be possibly loaded from memory |
|G3 | It will be possible to set the value of a particular configuration entry|
|G4 | It will be possible to get the value of a particular configuration entry |
|G5 | It will be possible to inspect the configured entries |
|G6 | More data types can be used as value for the different configurations entries |
|G7 | There will be a protection against changes to the configuration during flight |
|G8 | No dynamic allocations will be executed during the flight |
|G9 | The registry will offer a persistent configuration saving |
|G10 | It will be possible to verify the integrity of the configuration|
|G11 | It will be possible to explore the current configuration |
|G12 | Thread safeness is guaranteed |

**Note:** the Registry Frontend purpose is not to assure with 100% certainty that a configuration is saved in memory and will be loaded. Instead it is an
additional safety net which, in case of a restart of the rocket, could possibly lead to have the saved configuration re-loaded after reboot/reset.

This note about the registry frontend purpose also considers the fact that a configuration may not be saved in memory, that the memory could have issues or even that a configuration is saved but corrupted.

### Assumptions
The front-end, FE, considers some important assumptions about the usage of such sw component.
This assumptions are about the method call, the order of operations and other assumptions on the usage.
Such assumptions considers the actual necessities for Skyward Registry and might not be true in future.

| Assumption |  Description  |
|:-----|:--------:|
|A1 | The FE is constructed and started once, a single time|
|A2 | The FE does saves and loads a single configuration from the registry and no multiple versions|
|A3 | The FE is called before the flight phase for instantiation, load and set |
|A4 | The FE is correctly armed before the flight |
|A5 | The FE disarming is not used during flight phase |
|A6 | The caller considers the return of the get and set methods|
|A7 | Configuration entries does not have pointers as datatype |
|A8 | The backend does not modify the vector to be saved.|
|A9 | The save method is correctly trigger externally when a save is needed |

### Requirements

| Requirements |  Description  |
|:-----|:--------:|
|R1 | The interface must allow setting a value for the specific configuration entries |
|R2 | The interface must allow getting a specified value for a   configuration entries |
|R3 | The interface must perform some type check for the specifics entries to get |
|R4 | The interface must not allocate memory during the flight phase |
|R5 | The interface must not change the configuration entries during flight phase |
|R6 | The interface does save the configuration entries using the back-end components |
|R7 | The FE must manage concurrent get/set by multiple threads (Thread safety)|
|R8 | The FE must manage concurrent arm/disarm by multiple threads  (Thread safety)|
|R9 | The FE must allow exploration of the actual configuration|
|R10 | The FE should be able to control the configuration state (empty or has elements)|
| R11 | The FE should not be blocked during the actual backend saving procedure |

### Interface methods

The unsafe (type-unsafe) methods uses a parameter value for a specific registry entry uint32_t index identifier (could be a casted enumerator from OBSW structures) and a value from a specific data type. The Unsafeness is given by the fact that the data type is given from the value data type given to the set/get methods.

- `start`: Starts the backend and other objects that requires to be started, as an ActiveObject.

- `setUnsafe`:  A setter method is in need to ensure R1. This method should allow 
    insert a value for a specific configuration entry while guarantee the different data types for the values (R3).
    
- `getUnsafe`:  A getter method is in need to ensure the visibility of the configuration. 
    Such method should get a value for a specified configuration entry and changes the value to passed by reference value parameter. It does check that the type is consistent with the saved one.

- `getOrSetDefaultUnsafe`: A particular get method which returns the get value and if it is not existing in the configuration set it (if possible!) and returns the default value.

 - `arm`:  An "arm" method which guarantees no memory allocations and no changes to the configuration are in place
    until a "disarm" method. It is an important functionality to ensure a "safe" memory configuration during flight.

 - `disarm`: A "disarm" method, to disable the stable and "safe" configuration mode of the registry to allow again 
    allocations and settings on the configuration.

 - `isEmpty`: A method to know if there is an existing configuration or if it is empty

- `configuredEntries`: A method which returns the already existing entries of the configuration as a set.

- `load`: Loads from the backend the configuration. If no backend, it loads it from its own vector. 

- `save`: Saves the configuration to the backend.

- `clear`: Clears the configuration both in frontend and backend components, starting with an empty configuration.

- `forEach`: Given a callback, it does apply it for each element of the actual configuration with the id and EntryStructUnion parameters. 

### Data structures
The data structures are managed in 2 main header files.
#### RegistryTypes.h
Type structures have:

- `TypeUnion`: The union type for saving the values of the different configuration entries

- `EntryStructsUnion`: The structure actually saved into the configuration, with the value and type attributes. Also, it does expose all the useful operations for get/set the value from/to union and create a new instance of EntryStructsUnion through `make(value)` method.

#### RegistrySerializer.h

- `RegistryHeader`: The header structure for the serialization and de-serialization. Contains the attributes and information useful for the serialization and de-serialization procedures.
- `RegistryFooter`: Contains the CRC/Checksum of the serialized configuration.

## The serializer
The serializer simply taken a configuration, it serialize or deserialize it on the vector given in the constructor, following the format specified above. It isolates the serialize and de-serialize procedure from the frontend by making it simpler and more coherent.

### Exposed methods:
- `RegistrySerializer(vector)` The constructor needs the vector to/from which it writes the serialized data or reads to deserialize it.

- `serializeConfiguration(configuration)` Serializes the configuration + header and footer to the vector given in the constructor.

 - `deserializeConfiguration(configuration)` From the vector, loads the configuration inserting the entries if the vector checks (CRC, length, startBytes) are verified.

### Data saving serialization
serializationVector is a vector that after getSerializedVector will contain:
|||
|:-----|--------:|
|Header| serializardConfigurations|

As for now, the header (visible in RegistryTypes.h) is composed with 8B of bytes equal to decimal 1 for endianess checking, 4 bytes of vector length (whole vector including the header), 4B of nr. of configuration entries in the serialized vector, 4B of CRC - checksum computed with xor byte per byte of the serialized configurations following the header.

|||||||||
|:-----|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|----:|
|8B = 1| 4B Length vector | 4B Nr. entries | 4B CRC-Checksum | ID_0 | TypeID_0 | Value_0| ... |

Header closeup (0bit as rightmost one / big endian):

|||||||||||||||||||||
|:-----|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|----:|
|0|0|0|0|0|0|0|1|v_len 31-24|v_len 23-16|v_len 15-8|v_len 7-0|nr_en 31-24|nr_en 23-16|nr_en 15-8|nr_en 7-0|crc 31-24|crc 23-16|crc 15-8|crc 7-0|

The vector will contain only the set configurations. After the header, there will be all the configured entries with configuration ID, Type ID, Value(s).

In case of multiple values, they are one after the other 

e.g. for a Coordinate: 
|||||
|:-----|:--------:|:--------:|--------:|
| ID: 2 | TypeID: 1 | Val_1: 45.50109 | Val_2: 9.15633 | 

Where TypeID in this example is a coordinates type and therefore 45.50109 is the latitude and 9.15633 the longitude

### Saving
The save of the configuration is done manually by using the RegistryFrontend `save()` method. To reset the memory `clear()` + `save()` should be used.