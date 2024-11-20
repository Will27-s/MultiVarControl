import serial
import time
import csv
import os


ser = serial.Serial('COM8',9600)
time.sleep(2) # Gives time for the Arduino to initialise

# Set the duration to record data (in seconds)
record_duration = 10  # Record for 10 seconds
start_time = time.time()  # Record the current time (start time)

# Open a CSV file to record the data
file_name = "systemIdentification.csv"
dirname = os.path.dirname(__file__)
path = os.path.join(dirname,file_name)
    
with open(path, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)

    try:
        while True:
            # Check if the recording time has passed
            current_time = time.time()
            if current_time - start_time >= record_duration:
                print(f"Recording stopped after {record_duration} seconds.")
                break  # Stop the loop once the time is exceeded

            # Read incoming data from the serial port
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()  # Read and decode the serial data
                print(f"Received data: {line}")  # Optional: Print the data for debugging
                
                # Split the data if it has multiple values (optional, depending on how the data is sent)
                data_row = line.split(',')  # Assumes data is sent as comma-separated values

                # Write the data to the CSV file
                csvwriter.writerow(data_row)  # Write each data point or row to the CSV

    except KeyboardInterrupt:
        print("Recording stopped by user.")
        
# Close the serial connection
ser.close()



