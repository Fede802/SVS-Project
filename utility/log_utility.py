import os

class Logger:
    def __init__(self, log_dir = "logs", log_file_name_prefix = "log"):
        self.log_file = open(self.__compute_file_path(log_dir, log_file_name_prefix), "w")
    
    def __compute_file_path(self, log_dir, log_file_name_prefix):
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        files = os.listdir(log_dir)
        max_index = 0
        for f in files:
            log_number = int(f.split("_")[-1].split(".")[0])
            max_index = max(max_index, log_number)
        return log_dir+"/"+log_file_name_prefix + "_" + str(max_index + 1) + ".txt"    

    def write(self, message):
        self.log_file.write(message + "\n")
        
    def close(self):
        self.log_file.close()    