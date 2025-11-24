# P4 Global Navigation

## Introducción

La práctica consiste en la creación de un algoritmo de navegación global capaz de guiar a un taxi entre diferentes calles de una ciudad, evitando chocar con los edificios.

La aproximación a este problema se realizará mediante descenso por gradiente y la creación del mapa de rejillas pertinente para este.

![Introducción](Introduccion.png)

## Creación del mapa de costes
La primera parte de la práctica consiste en la creación de un mapa de costes. 

La principal idea en este punto es dar costes desde el objetivo, siendo este 0, hasta el taxi. Para lograr una distribución de los costes sobre el mapa, iremos aumentando el coste en 1 en las casillas adyacentes y $\sqrt{2}$ en las casillas diagonales desde el objetivo. 

Como es necesario recorrer todo el mapa, y no podemos ir casilla por casilla rellenando costes, he implementado un sistema de Nodo independientes que devuelven sus coordenadas, sus adyacentes y las diagonales cuando se llama a las funciones pertinentes. Esto facilita el manejo de las coordenadas, ya que cada una será un nodo con funciones.

### Experiencias y resultados

La idea era sencilla, para realizar la parte de distribución del coste, quise emplear la distancia euclideana entre dos puntos. Hayar de esta forma los costes desde el objetivo hasta el taxi iba a ser más fácil que ir sumando de uno en uno todo el rato.

**Formula empleada**

$d = \sqrt{(x2 - x1)^{2} + (y2 - y1)^{2}}$

El resultado no fue el esperado, puesto que los costes se distribuían en círculo alrededor del objetivo, ignorando muros.

![Distancia euclideana](Distancia_euclideana.png)

Al final, deje como implementación la descrita con anterioridad, sumar de 1 en 1 los costes y extender nodos al rededor del objetivo para ir guardando los costes en el mapa de costes.

## Obstáculos

Como nuestro mapa de costes no es sensible a los obstáculos, debemos crear otro mapa auxiliar que expanda los costes al rededor de cada obstáculo encontrado.

### Experiencias y resultados

La idea planteada es hacer una estrella _*_ al rededor del obstáculo en la cual los costos se amplian de manera progresiva desde afuera hasta el obstáculo. El coste a sumar es 60 de base, pero se divide por la distancia al obstáculo, haciendo que el coste sumado sea menor cuanto más lejos.

## Navegación

En esta parte se ha planteado el método de descenso por gradiente para navegar sobre el mapa.

La idea es tomar pasos de manera repetida en dirección contraria al gradiente. Esto se hace ya que esta dirección es la del descenso más empinado. Si se toman pasos con la misma dirección del gradiente, se encontrará el máximo local de la función y lo que nos interesa es llegar al mínimo local.

En nuestro caso, los costes menores serán el camino a seguir para llegar al objetivo.

### Experiencias y resultados

Para encontrar el punto con el coste más pequeño al rededor del coche, emplee una búsqueda en las casillas colindantes a 4 bloques de distancia.

![Busqueda coste menor](Busqueda_coste_menor.png)

De esta forma recorro una rejilla al rededor del coche y guardo la posición deseada para generar un vector de atracción con el que calcular la velocidad de giro para el taxi.

![Rejilla](Rejilla.png)

El planteamiento es simple, pero el manejo de las coordenadas en diferentes sistemas de referencia dificulta trabajar con ellas. Es importante conocer que las coordenadas del mapa se expresan en (y, x).

### Video del funcionamiento[^1]

[^1]: https://urjc-my.sharepoint.com/:v:/g/personal/e_martint_2022_alumnos_urjc_es/EQYiFQlWP25FnfpZnauXfzgBZnpDzYR39_nQs6MEyWa-Ag?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=a9hr9h

