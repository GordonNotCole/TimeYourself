#ifndef _RingBuf_h
#define _RingBuf_h
template < typename TYPE, uint8_t SIZE >
class RingBuf {
public:
    
    RingBuf(){
      head = 0;
      tail = 0;
    }
    bool write(TYPE newVal) {
 
        if (tail + 1 != head) {                
            buffer[tail++] = newVal;                   
            return 1;
        } else return 0;
    }

    bool availableForWrite() {
        return tail + 1 != head;
    }

    TYPE read() {
        if (head == tail) return 0;        
        return buffer[head++];                  
    }

    TYPE peek(){
        return buffer[tail];
    }
  
    uint8_t availableForRead() {
        return head != tail;
    }

private:
    TYPE buffer[SIZE];
    uint8_t head:5;
    uint8_t tail:5;

};
#endif