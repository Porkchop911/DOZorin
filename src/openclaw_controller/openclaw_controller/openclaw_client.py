#!/usr/bin/env python3
# encoding: utf-8
import os
import json
import requests
from openclaw_controller.config import *

class OpenClawClient:
    def __init__(self, config_path: str = "~/.openclaw/openclaw.json"):
        config_path = config_path.replace("~", os.path.expanduser("~"))
        with open(config_path) as f:
            config = json.load(f)
        
        self.gateway_url = f"http://127.0.0.1:{config['gateway']['port']}"
        self.token = config['gateway']['auth']['token']
        self.stream = False
        self.user = "robot_voice"
    
    def set_stream(self, stream: bool):
        self.stream = stream
    
    def chat(self, prompt: str) -> str:
        url = f"{self.gateway_url}/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {self.token}",
            "Content-Type": "application/json"
        }
        # system_prompt = (
        #     "1、你的回答风趣且变化无穷的反馈信息，让交流过程妙趣横生。"                                                                 
        #     "2、严格控制在10-20字以内，绝不超过20字" 
        # )
        data = {
            "model": "openclaw",
            "user": self.user,
            "messages": [
                # {"role": "system", "content": system_prompt},
                {"role": "user", "content": prompt}
                ],
            "stream": self.stream
        }
        
        if self.stream:
            response = requests.post(url, headers=headers, json=data, stream=True, timeout=120)
            full_content = ""
            for line in response.iter_lines():
                if line:
                    line = line.decode('utf-8')
                    if line.startswith('data: '):
                        try:
                            chunk = json.loads(line[6:])
                            content = chunk['choices'][0].get('delta', {}).get('content', '')
                            full_content += content
                        except:
                            pass
            return full_content
        else:
            response = requests.post(url, headers=headers, json=data, timeout=120)
            return response.json()['choices'][0]['message']['content']
