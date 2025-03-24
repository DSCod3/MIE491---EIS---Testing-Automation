from graphviz import Digraph

def main():
    dot = Digraph(comment='PLC Architecture Diagram', format='png')
    
    # Global graph attributes for a cleaner layout
    dot.attr(rankdir='LR',  # Layout from left-to-right
             splines='spline',  # Use smooth splines for edges
             nodesep='1.2',     # Increase node separation
             ranksep='1.0')   # Increase rank separation

    # Define nodes with descriptions
    dot.node('GUI', 'GUI on Laptop\n(WPH Interface)', shape='box', style='filled', fillcolor='lightblue')
    dot.node('Serial', 'Serial Communication\n(112500 bps)', shape='box', style='filled', fillcolor='lightgrey')
    dot.node('PLC', ('PLC (ESP32 based)\n'
                     '- Sensor Inputs:\n  • Potentiometers\n  • Load Cells\n  • Pressure Sensors\n'
                     '- Processing:\n  • PID Control\n  • Load Curve Execution\n  • Warmup Routine\n'
                     '- Serial Command Processing'), shape='box', style='filled', fillcolor='orange')
    dot.node('Valve', 'Analog Output\n(Valve Control via PID)', shape='box', style='filled', fillcolor='lightgreen')
    dot.node('Solenoids', 'Digital Outputs\n(Solenoid Valve Control)', shape='box', style='filled', fillcolor='lightgreen')
    dot.node('Air', 'Air Pistons\n(Load Cell & Pressure Feedback)', shape='box', style='filled', fillcolor='yellow')
    dot.node('Display', 'Data Logging & Display\n(WPH GUI Feedback)', shape='box', style='filled', fillcolor='lightblue')

    # Define edges (connections) with labels
    dot.edge('GUI', 'Serial', 'User Commands\nParameters & Requests', color='blue')
    dot.edge('Serial', 'PLC', 'Commands & Feedback', color='blue')
    dot.edge('PLC', 'Valve', 'PID Output\nControl Signal', color='red')
    dot.edge('PLC', 'Solenoids', 'Solenoid Control Signals', color='red')
    dot.edge('PLC', 'Air', 'Control & Sensor Requests', color='purple')
    dot.edge('Air', 'PLC', 'Pressure Sensor Feedback', color='purple')
    dot.edge('PLC', 'Display', 'Processed Data & Status', color='darkgreen')
    dot.edge('Display', 'GUI', 'Visualization Output', color='darkgreen')

    # Render the diagram to a file and open it
    dot.render('PLC_Architecture_Diagram', view=True)

if __name__ == '__main__':
    main()
