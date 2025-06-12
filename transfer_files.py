import os, shutil

date = input("Date to transfer: ")
read_folder = "data/" + date
filenames = os.listdir(read_folder)

for file in filenames:
    file_time = file[14:20]
    full_folder_name = date + "_" + file_time[:2] + "-" + file_time[2:4] + "-" + file_time[4:]

    full_sourcename = read_folder + "/" + file
    full_destination = "W:/Ephys/data_132F/" + date[:7] + "/" + date + "/" + full_folder_name + "/raw_data"
    os.makedirs(full_destination,exist_ok=True)
    shutil.copy2(full_sourcename,full_destination)

print("Done")
