import matplotlib.pyplot as plt


# Define the point class
class Point:
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t


# Sample points for closed_set and open_Set
closed_set = [Point(1, 2, 3), Point(2, 3, 4), Point(3, 4, 5)]
open_Set = [Point(4, 5, 6), Point(5, 6, 7), Point(6, 7, 8)]

# Extract coordinates
closed_x = [point.x for point in closed_set]
closed_y = [point.y for point in closed_set]
closed_t = [point.t for point in closed_set]

open_x = [point.x for point in open_Set]
open_y = [point.y for point in open_Set]
open_t = [point.t for point in open_Set]

# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot points
ax.scatter(closed_x, closed_y, closed_t, c="r", marker="o", label="Closed Set")
ax.scatter(open_x, open_y, open_t, c="b", marker="x", label="Open Set")

# Setting labels and title
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("T")
ax.set_title("3D Grid Map Visualization")
ax.legend()

# Show the plot
plt.show()
