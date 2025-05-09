import matplotlib.pyplot as plt

# Input cases
inputs = [1, 2, 3, 4]

# Metrics for each algorithm
time_uc = [0, 0, 74, 198]
time_mis = [0, 0, 11, 38]
time_man = [0, 0, 2, 5]

queue_uc = [8, 28, 6584, 18858]
queue_mis = [3, 6, 370, 2035]
queue_man = [3, 6, 63, 168]

# Plot 1: Time Taken Comparison
plt.figure()
plt.plot(inputs, time_uc, marker='o', label='Uniform Cost Search')
plt.plot(inputs, time_mis, marker='o', label='A* (Misplaced Tile)')
plt.plot(inputs, time_man, marker='o', label='A* (Manhattan Distance)')
plt.xlabel('Input Case Number')
plt.ylabel('Time Taken (ms)')
plt.title('Time Taken Comparison of Search Algorithms')
plt.legend()
plt.xticks(inputs)
plt.grid(True)

# Plot 2: Max Queue Size Comparison
plt.figure()
plt.plot(inputs, queue_uc, marker='o', label='Uniform Cost Search')
plt.plot(inputs, queue_mis, marker='o', label='A* (Misplaced Tile)')
plt.plot(inputs, queue_man, marker='o', label='A* (Manhattan Distance)')
plt.xlabel('Input Case Number')
plt.ylabel('Max Queue Size')
plt.title('Max Queue Size Comparison of Search Algorithms')
plt.legend()
plt.xticks(inputs)
plt.grid(True)

# Display both plots
plt.show()
