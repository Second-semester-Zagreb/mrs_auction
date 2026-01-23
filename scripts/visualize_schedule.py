#!/usr/bin/env python3
"""
Visualize auction schedule as a Gantt chart.
Usage: python3 visualize_schedule.py <schedule_json_file>
"""

import json
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from datetime import datetime
import os


def load_schedule(json_file):
    """Load schedule from JSON file."""
    with open(json_file, 'r') as f:
        return json.load(f)


def visualize_gantt(schedule, output_file=None):
    """Create and display/save Gantt chart from schedule."""
    
    # Extract robot IDs and sort them
    robots = sorted(schedule.keys())
    
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Color map for tasks
    colors = {}
    color_palette = plt.cm.Set3(range(20))
    travel_color = '#D3D3D3'  # Light gray for all travel times
    
    # Track which task labels we've added to avoid duplicates
    added_labels = set()
    
    # Plot bars for each robot
    y_pos = 0
    y_labels = []
    y_ticks = []
    
    for robot in robots:
        y_labels.append(robot)
        y_ticks.append(y_pos)
        
        robot_data = schedule[robot]
        task_names = robot_data['name']
        times = robot_data['time']
        
        current_time = 0.0  # Start from 0 for normalized timeline
        
        for task_name, (start, end) in zip(task_names, times):
            # Calculate travel time (gap before this task)
            travel_time = start - current_time
            
            if travel_time > 0:
                # Plot travel time segment
                ax.barh(y_pos, travel_time, left=current_time, height=0.6,
                       color=travel_color, edgecolor='black', linewidth=1.5,
                       label='Travel' if 'Travel' not in added_labels else "")
                if 'Travel' not in added_labels:
                    added_labels.add('Travel')
                
                # Add travel label if segment is wide enough
                if travel_time > 0.5:  # Only show text if segment is wide enough
                    mid_travel = current_time + travel_time / 2
                    ax.text(mid_travel, y_pos, 'Travel', ha='center', va='center',
                           fontsize=8, style='italic', alpha=0.7)
            
            # Update current time to task start
            current_time = start
            
            # Calculate task duration
            duration = end - start
            
            # Assign color to task if not already assigned
            if task_name not in colors:
                colors[task_name] = color_palette[len(colors) % len(color_palette)]
            
            # Plot task bar
            ax.barh(y_pos, duration, left=current_time, height=0.6,
                   color=colors[task_name], edgecolor='black', linewidth=1.5,
                   label=task_name if task_name not in added_labels else "")
            if task_name not in added_labels:
                added_labels.add(task_name)
            
            # Add task label in the middle of the bar
            mid_time = current_time + duration / 2
            ax.text(mid_time, y_pos, task_name, ha='center', va='center',
                   fontsize=9, fontweight='bold')
            
            # Update current time to task end
            current_time = end
        
        y_pos += 1
    
    # Set labels and title
    ax.set_yticks(y_ticks)
    ax.set_yticklabels(y_labels)
    ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Robots', fontsize=12, fontweight='bold')
    ax.set_title('Mission Schedule (with Travel Times)', fontsize=14, fontweight='bold')
    
    # Add grid
    ax.grid(True, axis='x', alpha=0.3, linestyle='--')
    
    # Create legend for tasks (Travel first, then tasks alphabetically)
    legend_items = []
    if 'Travel' in added_labels:
        legend_items.append(mpatches.Patch(facecolor=travel_color, edgecolor='black', label='Travel'))
    
    for task in sorted(colors.keys()):
        legend_items.append(mpatches.Patch(facecolor=colors[task], edgecolor='black', label=task))
    
    ax.legend(handles=legend_items, loc='upper right', bbox_to_anchor=(1.15, 1))
    
    # Adjust layout
    plt.tight_layout()
    
    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Gantt chart saved to {output_file}")
    else:
        plt.show()
    
    return fig, ax


def main():
    if len(sys.argv) < 2:
        # If no argument provided, try to find the most recent schedule file
        schedule_dir = os.path.join(os.path.dirname(__file__), '..', 'schedule')
        if os.path.exists(schedule_dir):
            files = sorted([f for f in os.listdir(schedule_dir) if f.startswith('schedule_') and f.endswith('.json')])
            if files:
                json_file = os.path.join(schedule_dir, files[-1])
                print(f"Using most recent schedule: {json_file}")
            else:
                print("No schedule files found in schedule directory")
                sys.exit(1)
        else:
            print(f"Usage: python3 visualize_schedule.py <schedule_json_file>")
            print(f"Schedule directory not found: {schedule_dir}")
            sys.exit(1)
    else:
        json_file = sys.argv[1]
    
    if not os.path.exists(json_file):
        print(f"Error: Schedule file not found: {json_file}")
        sys.exit(1)
    
    # Load schedule
    schedule = load_schedule(json_file)
    
    # Generate output filename
    base_name = os.path.splitext(os.path.basename(json_file))[0]
    output_file = os.path.join(os.path.dirname(json_file), f"{base_name}_gantt.png")
    
    # Create and save Gantt chart
    visualize_gantt(schedule, output_file)


if __name__ == '__main__':
    main()
