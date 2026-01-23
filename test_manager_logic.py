
import yaml
import json
import os

# Mock the manager's logic for loading tasks and publishing commands
def test_manager_logic():
    tasks_yaml_path = '/home/benz/Desktop/IFROS/MRS/Consensus_and_auction/mrs_auction/config/tasks.yaml'
    
    print(f"Loading {tasks_yaml_path}...")
    with open(tasks_yaml_path, 'r') as f:
        data = yaml.safe_load(f)
        tasks_data = {t['id']: t for t in data.get('tasks', [])}

    task_id = 'T8'
    if task_id not in tasks_data:
        print(f"Error: Task {task_id} not found!")
        return

    task_config = tasks_data[task_id]
    location = task_config.get('location', [0.0, 0.0])
    formation = task_config.get('formation', 'line')
    spacing = task_config.get('formation_spacing', 4.0)

    print(f"Task {task_id} loaded:")
    print(f"  Formation: {formation}")
    print(f"  Spacing: {spacing}")

    msg_dict = {
        'task_id': task_id,
        'location': location,
        'formation': formation,
        'spacing': float(spacing)
    }
    
    print("Generated JSON message:")
    print(json.dumps(msg_dict, indent=4))

    if spacing == 1.5 and formation == 'triangle':
        print("SUCCESS: Spacing and formation are correct.")
    else:
        print("FAILURE: Spacing or formation incorrect.")

if __name__ == "__main__":
    test_manager_logic()
