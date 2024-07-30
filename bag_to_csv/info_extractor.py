# Copyright 2024 National Research Council STIIMA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
