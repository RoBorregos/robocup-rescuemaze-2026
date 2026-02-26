template <typename T>
class arrCustom {
private:
    uint8_t i = 0;
    size_t size;
    T* positions;
    T dummy; // safe fallback for invalid access

public:
    // Constructor
    arrCustom(size_t size, T value) : dummy(value) {
        this->size = size;
        if (size <= 0) {
            this->size = 0;
            positions = new T[1]; // allocate at least 1 element for dummy
            positions[0] = value;
            return;
        }
        positions = new T[size]; 
        for(uint8_t i = 0; i < size; i++){
            positions[i] = value;
        }
    }
    
    void set(size_t index, T value) {
        if (index == 255) {
            Serial.println("SET: index is 255 (not found), skipping");
            return;
        }
        if (index < size) {
            positions[index] = value;
        } else {
            Serial.print("SET out of bounds - index: "); 
            Serial.print(index);
            Serial.print(" | size: ");
            Serial.println(size);
        }
    }
    
    uint8_t getIndex(T value) const {
        for(uint8_t j = 0; j < i; j++){ 
            if(positions[j] == value){
                return j;
            }
        }
        return 255; // kMaxInt = not found
    }
    
    // Safe getValue with dummy return
    T& getValue(size_t index) {
        if (index == 255) {
            Serial.println("GET: index is 255 - not in map!");
            return dummy;
        }
        if (index >= size) {
            Serial.print("GET out of bounds - index: ");
            Serial.print(index);
            Serial.print(" | size: ");
            Serial.print(size);
            Serial.print(" | used: ");
            Serial.println(i);
            return dummy;
        }
        return positions[index];
    }
    
    const T& getValue(size_t index) const {
        if (index == 255 || index >= size) {
            return dummy;
        }
        return positions[index];
    }
    
    size_t getSize() const { 
        return size; 
    }
    
    uint8_t getUsed() const { 
        return i; 
    }
    
    //just for som arrays using push_back
    void push_back(T position) {
        if (i >= size) {
            T* temp = new T[size + 1];
            for (uint8_t j = 0; j < size; j++) {
                temp[j] = positions[j];
            }
            delete[] positions;
            positions = temp;
            size++;
        }
        positions[i] = position;
        i++;
    }
    
    void reset() {
        i = 0;
        for (uint8_t j = 0; j < size; j++) {
            positions[j] = T();
        }        
    }
};