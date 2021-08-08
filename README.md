# CER Selbsttestfragen
Hier sind alle Fragen die am Ender jeder Vorlesung stehen gelistet und von mir Beantwortet. <br>
Alle Antworten sind nach besten Wissen und Gewissen, steinigt mich nicht wenn was Falsch ist <br>
Fragen von SoSe 21 <br>
Gruß geht raus an Jan, der die Fragen selbst formuliert aber nicht gescheit in den Foliensätzen beantwortet 

## Vorlesung 0 Grundlagen


### Was ist eine Simulation?
<details>
    <summary>Antwort</summary>
    Eine Simulation ist der Prozess der Modellierung eines echten System und die ausführung von virtuellen experimenten auf diesem. Zweck der Simulation ist entweder das Nachvolziehen des echten Systems oder das Entwickeln von Strategien für die Anwendung des echten Systems
</details>

### Warum und wofür braucht man Simulation?
<details>
    <summary>Antwort</summary>
    Das echte System ist ..
    <li>zu groß oder klein</li>
    <li>zu schnell oder zu langsam</li>
    <li>noch nicht geabaut</li>
    <li>zu gefährlich</li>
    <li>nicht experimentierbar</li>
    <li>zu teuer</li>
    <li>zu stark gestört</li>
</details>

### Was ist ein Modell?
<details>
    <summary>Antwort</summary>
    Im grunde Alles<br>
    Modellierung ist die Beschreibung der Realität. Dabei verbinden Modelle beobachtbare oder messbare Variabeln so das verständliche Muster entstehen
</details>

### Was haben Modelle mit Wissenschaft zu tun?
<details>
    <summary>Antwort</summary>
    Modelle Ermöglichen testbare Vorhersagen und somit die "Scientific Emthod"
</details>

### Mit welchen Schritten erstellt man Modelle für ein System?
<details>
    <summary>Antwort</summary>
    <li>Analytische Modellierung: Modellgleichung basierend auf Expertenwissen aufstellen</li>
    <li>Systemidentification: Bestimmen der Parameter durch Experimente</li>
</details>

### Warum braucht auch die Künstliche Intelligenz Simulationen?
<details>
    <summary>Antwort</summary>
    Künstliche Inteligenzen müssen Trainiert werden, dies passiert meist in einem Simulierten umfeld um die Effizienz zu erhöhen
</details>

### Wofür können Modelle noch genutzt werden?
<details>
    <summary>Antwort</summary>
    Blöde frage eigentlich:
    <li>Kunst</li>
    <li>Sofwareentwicklung</li>
    <li>Produktentwicklung</li>
    <li>Verhaltensprognose/-analyse</li>
    <li>Buchstäblich alles</li>
</details>

### Kannst Du das SIR Modell in Python programmieren?
<details>
    <summary>Antwort</summary>

```python
    import numpy as np
    import matplotlib.pyplot as plt
    def sir(u):
        s, i, r = u
        gamma = 0.17
        betta = 0.23
        du = [
            - (betta * s * i),
            betta * s * i - gamma * i,
            gamma * i
        ]
        du = np.array(du)
        return du

    u0 = [1.0, 0.001, 0.0]
    n = 200
    data = []
    u_old = u0
    data.append(u_old)

    for i in range(200):
        du = sir(u_old)
        tmp = u_old + du
        data.append(tmp)
        u_old = tmp
    plt.plot(data)
    plt.legend(["S","I","R"])
    plt.show()
```
</details>

### Wie könnte ein SIEARS (A=Asymptomatic, zweites S=Rückfälle) aussehen?
<details>
    <summary>Antwort</summary>
    Grundidee: neue konstante für rückfall (rho) und asympt (alpha)<br>
    mehr weiß ich auch nicht

</details>

## Vorlesung 1 Modellierung/ Vorwärtskinematik

### Was sind Eigenvektoren und wozu braucht man die?
<details>
    <summary>Antwort</summary>
    "lambda, v sin Eigenwert und der Dazugegörige Eigenvektor von A gdw Av = lambda v"
    "Eigenvektoren sind nützlich um Matrizen zu diagonalisieren" 

</details>

### Wie geht der Modellierungsloop?
<details>
    <summary>Antwort</summary>

```
    ˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍ                        ˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍ
    |Problem aus der Realität| -----Modellbildung---> |Mathematisches Problem|
    ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯                        ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
    ^                                                                   |
    |                                                                   |
    Überprüfung                                     Anwenden der Egrebnisse
    |                                               / Methoden
    |                                                                   |
    |                                                                   v
    ˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍ                       ˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍ
    |Sachverhalt in der Realität|<--Interpretation------|Lösung des Problems|
    ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯                       ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
```

</details>

### Wie unterscheiden sich statische und dynamische Modelle?
<details>
    <summary>Antwort</summary>
    Statische Modelle stellen Verschiedene Größen im Modell in Beziehung doer beschreibt eine abhängige Größe mit Hilfe einer anderen, das Model hängt also nur von diesen Größen ab<br>
    Dynamische Modelle hängen von dem jetzigen zustand, dem vorhergegangenen und diesen Werten ab (man betrachtet die ableitungen)
</details>

### Was bedeuten zeitdiskret, ortskontinuierlich, stochastisch?
<details>
    <summary>Antwort</summary>
    <li>zeitdiskret: zeitunabhängig</li>
    <li>ortskontinuierlich: Ortsabhängig</li>
    <li>stockastisch: wahrscheinlich? keine Ahnung steht nicht im Foliensatz</li>
</details>

### Welche Fragestellungen behandelt die Kinematik?
<details>
    <summary>Antwort</summary>
    <li>vorwärts: mit gegebenen Gelenkparameter q: wo steht unser Endeffektor</li>
    <li>rückwärts: mit welchen Gelenkparameter q erreicht unser Endeffektor die position x</li>
</details>

### Wie berechnet man Vorwärtskinematik?
<details>
    <summary>Antwort</summary>
    Pro gelenk wird das Ursprungskoordinatensystem entsprechend der Gelenkparameter rotiert und Verschoben. 
    Die Resultierende Matrix hat nun in den ersten drei Spalten und zeilen die Rotation des Endeffektors und in den letzen Spalte mit (x, y, z, 1)^T die Position des Endeffektors 
</details>

## Vorlesung 2 Inverse Kinematik

### Warum ist inverse Kinematik schwerer als Vorwärtskinematik?
<details>
    <summary>Antwort</summary>
    <li>Vorwärts: eine Explizite Lösung</li>
    <li>invers: unendlich viele Lösungen möglich</li>
</details>

### Wieviel Freitheitsgrade hat ein Auto?
<details>
    <summary>Antwort</summary>
    Drei Freiheitsgrade (auf Ebene Projiziert):
    <li>x</li>
    <li>y</li>
    <li>Fahrzeugorientierung</li>
</details>

### Wenn eine Schlange (mit 200-435 Wirbeln) sich aufrichtet, wiev iele inverse Kinematik-Lösungen hat sie?
<details>
    <summary>Antwort</summary>
    unendlich viele 
</details>


### Was macht das Fixpunktverfahren? Wann konvergiert es?
<details>
    <summary>Antwort</summary>
    <li>Das itterative anwenden einer Funktion auf sich selbst mit startwert x_0</li>
    <li>wenn d(g(x),g(y) <= kd(x, y) für alle x,y in R^n mit k in [0,1)</li>
    <li>also wenn alle eigenwerte der Jacobimatrix kleiner 1 (im Einheitskreis)</li>
</details>

### Was ist das Relaxationsverfahren? Wann ist es besser/schlechter als das Fixpunktverfahren?
<details>
    <summary>Antwort</summary>
    Das Relaxationsverfahren kann die Eigenwerte skallieren, sodass Sie im Einheitskreis liegen. Dies ermöglicht die Konvergenz<br>
    <li>besser: bestimmen von lokaler Konvergenz bei start x weit entfernt von Lösung</li>
    <li>schlechter: bestimmen der globalen Konvergenz</li> 
</details>


### Was macht das Newtonverfahren?
<details>
    <summary>Antwort</summary>
    Das Newtonverfahren ist eine Fixpunktiteration mit adaptiver Relaxationsmatrix
</details>

### Welche Verfahren sind 1. Ordnung, welche zweiter Ordnung?
<details>
    <summary>Antwort</summary>
    FPI und Relaxationsmatrix: 1. Ordnung <br>
    Newton: 2. Ordnung
</details>

### Wie kann ich das Newtonverfahren nutzen um ein Optimierungsproblem max_x s_{Note}(x) zu lösen? Z.B. für s_{Note}(x) = exp(−x) (1 − exp(−x))^{−2}? Tip: Nullstelle von s'_{Note}(x) = 0
<details>
    <summary>Antwort</summary>
    keine Ahnung 
</details>

## Vorlesung 3 Dynamische Modelle

### Was ein typisches Modell für die Federkraft?
<details>
    <summary>Antwort</summary>
    F<sub>Feder</sub> = k<sub>p</sub> (x<sub>ref</sub>-x) <br>
    mit k<sub>p</sub> als Federkonstante
</details>

### Was ein typisches Modell für Reibung?
<details>
    <summary>Antwort</summary>
    F<sub>Dämpfer</sub> = -k<sub>d</sub>x' <br>
    mit k<sub>d</sub> als Dämperfkonstante
</details>

### Wie wirken sich Kräfte auf die Position eines Körpers aus?
<details>
    <summary>Antwort</summary>
    Kräfte ändern die geschwindigkeit eines Körpers und somit auch die Position
</details>

### Wie wirken sich Kräfte auf die Orientierung eines Körpers aus?
<details>
    <summary>Antwort</summary>
    Garnicht, Drehmomente ändern die Orientierun
</details>

### Müssen wir bei Punktmassen die Orientierung modellieren?
<details>
    <summary>Antwort</summary>
    Nein
</details>

### Was ist die Dynamikgleichung eines Feder-Masse-Dämpfer Systems?
<details>
    <summary>Antwort</summary>

</details>

### Was ist die Dynamikgleichung eines Pendels?
<details>
    <summary>Antwort</summary>
    <img src="bilder/pendel.PNG" alt="Pendel">
</details>

### Wie kann ich ein einfaches mechanisches System modellieren?
<details>
    <summary>Antwort</summary>

</details>