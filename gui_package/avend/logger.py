#!/usr/bin/env python3

import logging
import os
from datetime import datetime

class AvendLogger:
    def __init__(self, log_level="INFO", log_to_file=True, log_file_path=None):
        self.logger = logging.getLogger("avend_connector")
        
        # Set log level
        level = getattr(logging, log_level.upper(), logging.INFO)
        self.logger.setLevel(level)
        
        # Create formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        # Add console handler
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)
        
        # Add file handler if enabled
        if log_to_file:
            if log_file_path is None:
                log_dir = "logs"
                os.makedirs(log_dir, exist_ok=True)
                log_file_path = os.path.join(
                    log_dir,
                    f"avend_connector_{datetime.now().strftime('%Y%m%d')}.log"
                )
            
            file_handler = logging.FileHandler(log_file_path)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
    
    def info(self, message):
        self.logger.info(message)
    
    def error(self, message):
        self.logger.error(message)
    
    def warning(self, message):
        self.logger.warning(message)
    
    def debug(self, message):
        self.logger.debug(message)
