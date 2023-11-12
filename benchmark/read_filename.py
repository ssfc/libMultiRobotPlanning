import os
import pandas as pd

folder_path = r'C:\gitcloud\libMultiRobotPlanning\benchmark\8x8_obst12'
file_list = os.listdir(folder_path)

df = pd.DataFrame(file_list, columns=['File Names'])

df.to_excel('filename.xlsx', index=False)
