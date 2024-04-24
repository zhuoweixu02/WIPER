import matplotlib.pyplot as plt
import tkinter as tk
import queue

# visualize_map:
# plots out the map to verify it looks alright
# def visualize_map(map_corners,plot_para):
#     center_x = plot_para[0]
#     center_y = plot_para[1]
#     ox = plot_para[2]
#     oy = plot_para[3]
#     boundary_xs = plot_para[4]
#     boundary_ys = plot_para[5]

#     fig, ax = plt.subplots()
#     ax.set_aspect('equal')
#     ax.grid(True, which='both', linestyle='--', linewidth=0.5)

#     # Set axis limits dynamically based on the corners
#     offset_grid = 10  # Additional space around the plotted area
#     resolution = 0.1
#     ax.set_xlim([min(ox) - offset_grid * resolution, max(ox) + offset_grid * resolution])
#     ax.set_ylim([min(oy) - offset_grid * resolution, max(oy) + offset_grid * resolution])

#     # Plot tag corners and fill exclusion zones
#     for tag_id, corners in map_corners.items():
#         corner_xs, corner_ys = zip(*[(corner['x'], corner['y']) for corner in corners])
#         ax.plot(corner_xs + (corner_xs[0],), corner_ys + (corner_ys[0],), '-o', label=f'Tag {tag_id}')  # Close the loop for squares
#         ax.fill(corner_xs, corner_ys, alpha=0.2)  # Fill exclusion zones

#     ax.plot(boundary_xs, boundary_ys, 'k--', linewidth=2, label='Map Boundary')
#     ax.plot(center_x, center_y, 'ro', label='Map Center')

#     ax.legend(loc='upper right')
#     plt.xlabel('X position (m)')
#     plt.ylabel('Y position (m)')
#     plt.title('Map Visualization with Exclusion Zones around Tags')
#     plt.show()



class MapVisualizer:
    def __init__(self, root):
        self.root = root
        self.canvas_width = 600
        self.canvas_height = 600
        self.data_queue = queue.Queue()

        # Initialize the canvas
        self.canvas = tk.Canvas(root, width=self.canvas_width, height=self.canvas_height, bg='white')
        self.canvas.pack()

        # Set window title
        self.root.title("Map Visualization")

        # Start the update checker
        self.check_for_updates()

    def scale_coord(self, x, y):
        """ Scale map coordinates to fit the canvas """
        scale = 50  # Adjust scale factor as needed
        return (x * scale + 50, y * scale + 50)

    def draw_map(self, map_corners, plot_para):
        """ Draw the boundary, tags, and center point on the canvas """
        center_x, center_y, ox, oy, boundary_xs, boundary_ys = plot_para

        # Clear the previous map
        self.canvas.delete("all")

        # Draw boundary
        scaled_boundary_coords = [self.scale_coord(x, y) for x, y in zip(boundary_xs, boundary_ys)]
        self.canvas.create_polygon(*scaled_boundary_coords, outline='black', fill='', dash=(4, 2))

        # Draw tags and exclusion zones
        for tag_id, corners in map_corners.items():
            scaled_corners = [self.scale_coord(corner['x'], corner['y']) for corner in corners]
            xs, ys = zip(*scaled_corners)
            self.canvas.create_polygon(*scaled_corners, outline='blue', fill='lightblue', tags=f'Tag {tag_id}')
            self.canvas.create_text(sum(xs)/len(xs), sum(ys)/len(ys), text=f'Tag {tag_id}')

        # Draw the center point
        center_coords = self.scale_coord(center_x, center_y)
        self.canvas.create_oval(center_coords[0]-5, center_coords[1]-5, center_coords[0]+5, center_coords[1]+5, fill='red')

    def check_for_updates(self):
        """ Check the queue for new data and update the map if necessary """
        try:
            map_corners, plot_para = self.data_queue.get_nowait()
            self.draw_map(map_corners, plot_para)
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.check_for_updates)

if __name__ == "__main__":
    root = tk.Tk()
    map_corners = {
        1: [{'x': 1, 'y': 1}, {'x': 2, 'y': 1}, {'x': 2, 'y': 2}, {'x': 1, 'y': 2}],
        2: [{'x': 3, 'y': 3}, {'x': 4, 'y': 3}, {'x': 4, 'y': 4}, {'x': 3, 'y': 4}]
    }
    plot_para = [2.5, 2.5, [0, 5], [0, 5], [0, 0, 5, 5], [0, 5, 5, 0]]
    app = MapVisualizer(root, map_corners, plot_para)
    root.mainloop()