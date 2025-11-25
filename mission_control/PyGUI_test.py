import PySimpleGUI as sg

layout = [[sg.Text(text='MODZONNAH',
                   font=('Arial Bold', 20),
                   size=20,
                   expand_x=True,
                   justification='center')],
          [sg.Button("OK")]]

window = sg.Window('Ciao un cazzo0', layout)

while True:
    event, values = window.read()
    print(event, values)
    if event == "OK" or event == sg.WIN_CLOSED:
        break
    
window.close()   

