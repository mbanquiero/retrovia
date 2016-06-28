#Taller de Matemática y Física para videojuegos
Dentro del marco **Jornada Matemática en Movimiento FOMA 2016** en la UTN Buenos Aires
[Noticia de FOMA - UTNBA En Movimiento](https://blog.frba.utn.edu.ar/utnbaenmovimiento/jornada-matematica-en-movimiento-foma-2016-en-la-utn-buenos-aires/)


##Referencia a materia de Técnicas Gráficas por Computador
[Link a la materia](tgcutn.com.ar)
[Ejemplos en JS](mbanquiero.github.io) 


###Oradores:
[Ing. Mariano Banquiero](https://github.com/mbanquiero) <mariano@lepton.com.ar>

[Ing. Rodrigo Nicolas Garcia](https://github.com/mysery) <rgarcia@frba.utn.edu.ar>

##Modo de uso del mini motor de fisica para ejemplos.
El motor esta realizado con Java Script y puede ser ejecutado en Chrome para ver sus resultados.

####Definir la gravedad:

    gravity = new Vector2(0, 10);

####Agregar una caja:

    world.AddBox(X, Y, DX, DY);

####Agregar un circulo:

    world.AddCircle(X, Y, radio);

####Agregar un triangulo:

    world.AddTri(ax, ay, bx, by, cx, cy);

####Definir un cuerpo estatico:

    piso.SetStatic();

####Definir la orientacion de un cuerpo.

    canion.SetOrient(radianes);



