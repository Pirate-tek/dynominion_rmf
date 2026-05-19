import yaml
with open('dynominion_rmf_maps/building/new_env.building.yaml') as f:
    data = yaml.safe_load(f)
v = data['levels']['L1']['vertices']
f = data['levels']['L1']['floors'][0]['vertices']
polygon = [(v[i][0], v[i][1]) for i in f]
def point_in_polygon(x, y, poly):
    n = len(poly)
    inside = False
    p1x, p1y = poly[0]
    for i in range(1, n + 1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

gz_poly = [(2.975 - px, py) for px, py in polygon]
print("Gazebo spawn 2 (1.48, -9.5) in Gazebo polygon:", point_in_polygon(1.48, -9.5, gz_poly))
print("Proper Gazebo point 2 (1.495, -9.5) in Gazebo polygon:", point_in_polygon(1.495, -9.5, gz_poly))
