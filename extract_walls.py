import xml.etree.ElementTree as ET

tree = ET.parse('/home/matteo/ros2_ws/turtlebot3_custom_simulation/worlds/casa.world')
root = tree.getroot()
world = root.find('world')
for model in world.findall('model'):
    mname = model.get('name')
    mpose_el = model.find('pose')
    mpose = mpose_el.text if mpose_el is not None else '0 0 0 0 0 0'
    print(f'MODEL: {mname}  model_pose={mpose}')
    for link in model.findall('link'):
        lname = link.get('name')
        lpose_el = link.find('pose')
        lpose = lpose_el.text if lpose_el is not None else '0 0 0 0 0 0'
        col = link.find('collision')
        if col is not None:
            box = col.find('.//box/size')
            if box is not None:
                print(f'  link={lname}  link_pose={lpose}  box={box.text}')
