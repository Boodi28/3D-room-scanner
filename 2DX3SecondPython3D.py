## Title Open3d Create Test Data
## Edited by: Abdalla Mahdy, mahdya, 400411114


import numpy
import open3d

if __name__ == "__main__":
    # Get the number of scans from the user
    scans = int(input("Enter the number of scans: "))
    spins = 16
    
    # Read the point cloud data from the file
    print("Reading point cloud data from file")
    pcd = open3d.io.read_point_cloud("3Dpoints.xyz", format="xyz")

    # Display the numerical representation of the point cloud data
    print("Point cloud data as an array:")
    print(numpy.asarray(pcd.points))

    # Visualize the point cloud data
    print("Displaying point cloud data (a new window will appear)")
    open3d.visualization.draw_geometries([pcd])

    # Give each vertex a unique identifier
    yz_slice_vertex = []
    for x in range(0,scans*spins):
        yz_slice_vertex.append([x])

    # Define the coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(0,scans*spins,spins):
        for i in range(spins):
            if i==spins-1:
                lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x]])
            else:
                lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+1]])
            
    # Define the coordinates to connect lines between the current and next yz slice
    for x in range(0,scans*spins-spins-1,spins):
        for i in range(spins):
            lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+spins]])

    # Map the lines to the 3D coordinate vertices
    line_set = open3d.geometry.LineSet(points=open3d.utility.Vector3dVector(numpy.asarray(pcd.points)),lines=open3d.utility.Vector2iVector(lines))

    # Visualize the point cloud data with lines
    open3d.visualization.draw_geometries([line_set]) 
