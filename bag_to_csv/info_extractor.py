from abc import ABC, abstractmethod

class InfoExtractor(ABC):
    def __init__(self, expected_msg_type):
        self.expected_msg_type = expected_msg_type

    def _check_msg_type(self, msg_type):
        if msg_type != self.expected_msg_type:
            raise ValueError(f'Unexpected msg_type: expected {self.expected_msg_type} got {msg_type}')
    
    def get_msg_type(self):
        return self.expected_msg_type
    
    def set_msg_type(self, expected_msg_type):
        self.expected_msg_type = expected_msg_type
    
    @abstractmethod
    def extract_info_from_msg(self, msg, msg_type):
        pass
