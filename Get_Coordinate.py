# Importing csv module to read the CSV file
import csv

'''
This code reads a CSV file and creates a dictionary to store node coordinates.
It uses the csv module to read the CSV file. 
The next() method is used to skip the header row of the file. 
Then it iterates over each row using a for loop, extracts the ID, latitude, and longitude values from the row, converts the latitude and longitude values to float type, and stores the ID with its respective latitude and longitude as a tuple in the coordinates dictionary.
Finally, it prints the dictionary of coordinates to the console.
'''

# Creating an empty dictionary to store node coordinates
coordinates = {}

# Reading CSV file using DictReader method and assigning field names
with open('tblnodes_202305111211.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile, fieldnames=['id', 'x', 'y'])

    next(reader)  # Skipping header row

    # Iterating over each row in CSV file using for loop
    for row in reader:
        id_ = row['id']                   # ID of current node
        x = float(row['x'])               # Latitude coordinate of current node
        # Longitude coordinate of current node
        y = float(row['y'])

        # Storing node coordinates with their respective IDs into dictionary
        coordinates[id_] = (x, y)

# Printing the dictionary of node coordinates
print(coordinates)
