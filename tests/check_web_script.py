import re, subprocess
from pathlib import Path
for variant in ('No_Encoders','Rear_Encoders'):
    s=Path(f'firmware/{variant}/SwarmRobot/SwarmRobot.ino').read_text()
    m=re.search(r'static const char PAGE\[\] PROGMEM = R"HTML\((.*?)\)HTML";',s,re.S)
    assert m, 'HTML missing'
    html=m.group(1)
    js=re.search(r'<script>(.*?)</script>',html,re.S)
    assert js, 'JavaScript missing'
    subprocess.run(['node','--check','-'],input=js.group(1),text=True,check=True)
    assert all(e in html for e in ('/api/drive','/api/status','/api/stop','/api/move'))
    assert 'n.rearEncoders' in html and 'canGoal' in html
    assert s.count('REAR_ENCODER_FLAG')>=3
    print(f'PASS {variant}: dashboard JS syntax, endpoints, capabilities')
