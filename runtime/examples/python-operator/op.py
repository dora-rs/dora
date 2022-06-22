def dora_init_operator():
    return Operator()

class Operator:
    counter = 0

    def dora_drop_operator(self):
        print('drop python operator')
    
    def dora_on_input(self, id, value, send_output):
        val_len = len(value)
        print(f'PYTHON received input {id}; value length: {val_len}')
        send_output("counter", self.counter.to_bytes(1, 'little'))
        self.counter = (self.counter + 1) % 256
