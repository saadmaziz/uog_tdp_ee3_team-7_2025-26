import pandas as pd

# Define the digitized data from the motor performance plot
data = {
    "Torque (Kgcm)": [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5],
    "Pout (Output Power)": [0.00, 0.85, 1.55, 2.10, 2.45, 2.65, 2.55, 2.20, 1.60, 0.00],
    "Amp (Current)": [0.10, 0.32, 0.53, 0.74, 0.95, 1.16, 1.37, 1.58, 1.79, 2.00],
    "Eff (Efficiency)": [0.20, 0.44, 0.42, 0.38, 0.34, 0.30, 0.24, 0.18, 0.12, 0.00],
    "kRPM (Speed)": [0.200, 0.178, 0.155, 0.133, 0.111, 0.089, 0.067, 0.044, 0.022, 0.000]
}

# Create a DataFrame
df = pd.DataFrame(data)

# File name for the output
file_name = "motor_performance_data.csv"

try:
    # Save to CSV
    df.to_csv(file_name, index=False)
    print(f"Successfully created {file_name}")
    
    # Optional: Display the first few rows of the generated DataFrame
    print("\nData Preview:")
    print(df.head())

except Exception as e:
    print(f"An error occurred while creating the CSV file: {e}")