
/**
 * @fileoverview body.js
<P> Implementa formas geometricas basicas</P>
<P>  -circulo</P>
<P>  -caja</P>
<P>  -triangulo</P>
<P>  -poligono</P>
<P>  Implementa dinamica de cuerpo rigido</P>
*/

var MAX_VERTEX_COUNT	= 32;
var GPE_CIRCLE		    = 0;
var GPE_POLYGON			= 1;


// Conviene desacoplar la "geometria" del objeto, de la clase body pp dicha. 
// Las clases geometrias encapsulan las funciones de computar masa, dibujar, 
// y demas que hacen a la geometria del cuerpo.

/**
 * GeoCircle
 * @class Shape circulo
 * @constructor
 * @param {float} r radio
 */
function GeoCircle(r)
{
    this.matWorld = new Matrix2(1,0,0,1);
    this.radius = r;
    this.drawCM = true;             // dibujo el centro de masa
    this.drawOrient = true;         // dibujo la rayita del orient
	this.cm = new Vector2(0,0);		// centro de masas
}

/**
 * Computo los inversos de la masa y la inercia. En la mayor parte de las ecuaciones de la dinamica de cuerpos rigidos
 * La masa y la inercia aparecen dividiendo, con lo cual es mas practico almacenar 1/mass y 1/inertia 
 * @method ComputeMass
 * @param {float} density densidad 
 * @param {Body} body rigid body asociado
*/
GeoCircle.prototype.ComputeMass = function(density , body)
{
    var r2 = this.radius * this.radius;
    body.mass = Math.PI * r2 * density;
    if(body.mass!=0)
        body.inverseMass = 1 / body.mass;
    // Momento de inercia
    body.inertia = body.mass * r2;
    if(body.inertia!=0)
        body.inverseInertia = 1 / body.inertia;


}

/**
 * Computo el bounding rect del cuerpo 
 * @method ComputeSize
 * @param {Body} body rigid body asociado
*/
GeoCircle.prototype.ComputeSize = function (body) {
    body.width = body.height = 2 * this.radius;
}


/**
 * Orientacion 
 * @method SetOrient
 * @param {float} angle angulo en radianes
*/
GeoCircle.prototype.SetOrient = function(angle) 
{
    this.matWorld.Rotation(angle);
}

/**
 * Dibuja por pantalla. 
 * Nota: Usa las variables globales ox,oy,ex,ey, y ctx (canvas)
 * @method Render
 * @param {Body} body rigid body asociado
*/
GeoCircle.prototype.Render = function(body)
{
    var pos = body.position;
    var r = this.radius;
    // dibujo un circulo en la posicion del centro de masas del cuerpo
    ctx.fillStyle = body.color;
    ctx.strokeStyle = '#ff00ff';
    ctx.beginPath();
    ctx.arc(ox + pos.x*ex, oy + pos.y*ey, r*ex, 0, Math.PI*2,true); 
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // dibujo una linea para captar la orientacion del circulo
    if (this.drawOrient) {
        ctx.beginPath();
        ctx.strokeStyle = '#ff0000';
        ctx.moveTo(ox + pos.x * ex, oy + pos.y * ey);
        var x = pos.x + Math.cos(body.orient) * r;
        var y = pos.y + Math.sin(body.orient) * r;
        ctx.lineTo(ox + x * ex, oy + y * ey);
        ctx.closePath();
        ctx.stroke();
    }
    // dibujo el centro de gravedad
    if(this.drawCM &&  body.inverseMass!=0)
        drawCenterOfMass(pos.x, pos.y, 0.5, body.orient);


}

/**
 * @method GetType
 * @return {integer} GPE_CIRCLE
*/

GeoCircle.prototype.GetType = function() {
    return GPE_CIRCLE;
}

/**
 * GeoPolygon
 * Crea un poligono vacio
 * @class Shape Poligono Convexo
 * @constructor
 */
function GeoPolygon()
{
    this.m_vertexCount = 0;
    this.vertices = [];
    this.normals = [];
	this.cm = new Vector2(0,0);		// centro de masas
	
}

/**
 * GeoPolygon
 * Crea una caja rectangular
 * @method
 * @param {float} w Width
 * @param {float} h Height
 */
GeoPolygon.prototype.CreateBox = function(w,h) 
{
    this.m_vertexCount = 4;
    var hw = w/2;			// half width
    var hh = h/2;			// half height
    this.vertices[0] = new Vector2( -hw, -hh );
    this.vertices[1] = new Vector2( hw, -hh );
    this.vertices[2] = new Vector2(  hw,  hh );
    this.vertices[3] = new Vector2( -hw,  hh );
    this.normals[0] = new Vector2(  0.0,  -1.0 );
    this.normals[1] = new Vector2(  1.0,   0.0 );
    this.normals[2] = new Vector2(  0.0,   1.0 );
    this.normals[3] = new Vector2( -1.0,   0.0 );
    this.matWorld = new Matrix2(1,0,0,1);
    this.Close();
}

/**
 * GeoPolygon
 * Crea un triangulo
 * @method
 * @param {Vector2} p1 Punto1
 * @param {Vector2} p2 Punto2
 * @param {Vector2} p3 Punto3
 */
GeoPolygon.prototype.CreateTri = function(p1,p2,p3)
{
    this.m_vertexCount = 3;
    this.vertices[0] = p1.Clone();
    this.vertices[1] = p2.Clone();
    this.vertices[2] = p3.Clone();
    this.normals[0] = vec2_normal(vec2_substract(p1,p2));
    this.normals[1] = vec2_normal(vec2_substract(p2,p3));
    this.normals[2] = vec2_normal(vec2_substract(p3,p1));
    this.normals[0].normalize();
    this.normals[1].normalize();
    this.normals[2].normalize();
    this.matWorld = new Matrix2(1,0,0,1);
    this.Close();
}

/**
 * GeoPolygon
 * Crea un Convex HULL a partir de una nube de puntos
 * @method
 * @param {Vector2[]} p Array de puntos
 * @param {int} count Cantidad de puntos del Array
 */
GeoPolygon.prototype.CreatePoly = function(p,count)
{
    // copio los vertices
    this.m_vertexCount = count;
    for(var i = 0; i < count; ++i)
        this.vertices[i] = p[i].Clone();
    // cierro la figura
    this.vertices[count] = p[0].Clone();

    // calculo las normales
    for(var i = 0; i < count; ++i)
    {
        var face = vec2_substract(this.vertices[i], this.vertices[i + 1]);
        this.normals[i] = vec2_normal(face);
        this.normals[i].normalize( );
    }
    this.normals[count] = this.normals[0].Clone();
    this.matWorld = new Matrix2(1,0,0,1);
}

/**
 * Cierra el poligono
 * @method Close
 */
GeoPolygon.prototype.Close = function()
{
    this.vertices[this.m_vertexCount] = this.vertices[0].Clone();
    this.normals[this.m_vertexCount] = this.normals[0].Clone();
}


/**
 * Orientacion 
 * @method SetOrient
 * @param {float} angle angulo en radianes
*/
GeoPolygon.prototype.SetOrient = function (angle) {
    this.matWorld.Rotation(angle);
}

/**
 <P> Computo los inversos de la masa y la inercia. En la mayor parte de las ecuaciones de la dinamica de cuerpos rigidos
  La masa y la inercia aparecen dividiendo, con lo cual es mas practico almacenar 1/mass y 1/inertia </P>
 <P>  Calcula el area  </P> 
 <P>  http://en.wikipedia.org/wiki/Centroid</P> 
 <P>  y el momento de inercia</P> 
 <P>  http://en.wikipedia.org/wiki/Second_moment_of_area</P> 
 <P>  formulas y ecuaciones </P> 
 <P>  http://richardson.eng.ua.edu/Former_Courses//CE_331_fa09/Projects/A_and_I_of_Polygon.pdf</P> 
 <P> surveyor-s formula para el calculo de area de un poligono</P> 
 <P>  http://steiner.math.nthu.edu.tw/disk5/js/cardioid/12.pdf</P> 
 *
 * @method ComputeMass
 * @param {float} density densidad 
 * @param {Body} body rigid body asociado
*/

GeoPolygon.prototype.ComputeMass = function( density , body)
{


    // Se supone que el pto N = pto 0 , ya que el poligono es cerrado
    this.vertices[this.m_vertexCount] = this.vertices[0].Clone();

    var c = new Vector2( 0.0, 0.0 ); // centro de masa
    var area = 0.0;
    var Ixy = 0.0;

    for(var i = 0; i < this.m_vertexCount; ++i)
    {
        var p1 = this.vertices[i];
        var p2 = this.vertices[i+1];
        var D = cross_vec( p1, p2 );
        var triangleArea = 0.5 * D;
        area += triangleArea;
        c.add( vec2_multiply(vec2_add(p1 , p2) , (triangleArea / 3.0)));
        var xx = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
        var yy = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
        Ixy += (0.25 * D / 3.0) * (xx + yy);
    }
    c.multiply(1/area);
    
    body.mass = density * area;
    if(body.mass!=0)
        body.inverseMass = 1 / body.mass;

    // Momento de inercia
    body.inertia = density * Ixy;
    if(body.inertia!=0)
        body.inverseInertia = 1 / body.inertia;

	
    // Ajusto la posicion de los vertices para que la posicion de origen (0,0) coincida con el centro de masas
    for(var i = 0; i <= this.m_vertexCount; ++i)
    {
        this.vertices[i].x -= c.x;
        this.vertices[i].y -= c.y;
    }
	
	this.cm.x = c.x;
	this.cm.y = c.y;
	
}

/**
 * Setea el centro de masas en un lugar diferente del (0,0)
 * @method SetCenterOfMass
 * @param {Vector2} c centro de masas
*/
GeoPolygon.prototype.SetCenterOfMass = function( c)
{
    // Ajusto la posicion de los vertices para que la posicion de origen (0,0) coincida con el centro de masas
    for(var i = 0; i <= this.m_vertexCount; ++i)
    {
        this.vertices[i].x -= c.x;
        this.vertices[i].y -= c.y;
    }
}

/**
 * Computo el bounding rect del cuerpo 
 * @method ComputeSize
 * @param {Body} body rigid body asociado
*/

GeoPolygon.prototype.ComputeSize = function (body)
{
    var min_x = 10000;
    var min_y = 10000;
    var max_x = -10000;
    var max_y = -10000;

    for (var i = 0; i <= this.m_vertexCount; ++i) {
        var x = this.vertices[i].x;
        var y = this.vertices[i].y;
        if (x < min_x)
            min_x = x;
        if (x > max_x)
            max_x = x;

        if (y < min_y)
            min_y = y;
        if (y > max_y)
            max_y = y;
    }
    body.width = max_x - min_x;
    body.height = max_y - min_y;
}

/**
 * Dibuja por pantalla. 
 * Nota: Usa las variables globales ox,oy,ex,ey, y ctx (canvas)
 * @method Render
 * @param {Body} body rigid body asociado
*/
GeoPolygon.prototype.Render = function(body)
{
    // dibujo un poligono
    ctx.fillStyle = body.color;
    ctx.strokeStyle = '#ff00ff';
    ctx.beginPath();
    for(var i = 0; i <= this.m_vertexCount; ++i)
    {
        var pos = vec2_add(body.position  ,  vec2_transform(this.matWorld , this.vertices[i]));
        if(i==0)
            ctx.moveTo(ox + pos.x*ex, oy+pos.y*ey); 
        else
            ctx.lineTo(ox + pos.x*ex, oy+pos.y*ey); 
    }
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // dibujo el centro de gravedad
    if (this.drawCM &&  body.inverseMass != 0)
        drawCenterOfMass(body.position.x, body.position.y, 0.5, body.orient);

}

/**
 * @method GetType
 * @return {integer} GPE_POLYGON
*/
GeoPolygon.prototype.GetType = function () {
    return GPE_POLYGON;
}

// helper para algortimo de colision convexo - convexo 
// devuele la el vertice que esta mas lejos en distancia proyectada en direccion dir
GeoPolygon.prototype.getMaxProjection = function (dir)
{
    var maxProjection = -10000;
    var rta;
    for(var i = 0; i < this.m_vertexCount; ++i)
    {
        var v = this.vertices[i];
        var projection = dot( v, dir );

        if (projection > maxProjection)
        {
            rta = v;
            maxProjection = projection;
        }
    }
    return rta;
}


/**
 * @class Body
 * Objeto rigid body. La escena esta compuesta de un cierto número de rigid body que interactuan entre si.
 * @property {Object} shape Forma del objeto 
 * @property {float} width Ancho del bounding rect
 * @property {float} height Alto del bounding rect
 * @property {Vector2} position Posicion
 * @property {Vector2} velocity Velocidad (lineal)
 * @property {Vector2} force Fuerza resultante
 * @property {float} angularVelocity Velocidad angular
 * @property {float} torque Tensor de inercia
 * @property {float} orient Angulo de orientacion
 * @property {float} density Densidad 
 * @property {float} inertia Momento de inercia
 * @property {float} inverseInertia 1/inertia
 * @property {float} mass masa
 * @property {float} inverseMass 1/masa
 * @property {float} gravity_factor factor que multiplica la fuerza de gravedad
 * @property {float} staticFriction Coeficiente de friccion estatica
 * @property {float} dynamicFriction Coeficiente de friccion dinamica
 * @property {float} restitution Coeficiente de restitucion
 * @property {float} maxForce maxima fuerza que resiste (resistencia del material)
 * @property {string} name nombre del objeto
 * @property {string} color color del objeto
 * @property {int} id id numérico
 * @property {Image} img Objeto imagen asociado
 * @property {boolean} deleted flag que indica que el objeto esta borrado
 * @property {boolean} visible flag que indica que si objeto es visible
 * @constructor
*/
function Body()
{
    // propiedades geometricas
    this.shape = null;
    this.width;
    this.height;

    // propiedades cinemáticas
    this.position = new Vector2(0,0);
    this.velocity = new Vector2(0,0);
    this.force = new Vector2(0,0);

    // oriented rigid body
    this.angularVelocity = 0;
    this.torque = 0;
    this.orient = 0;					// angulo de rotacion

    // mass properties
    this.density = 1;
    this.inertia = 0;					// momento de inercia
    this.inverseInertia = 0;			// inversa del momento de inercia = 1/inertia
    this.mass = 0;						// masa
    this.inverseMass = 0;				// inversa de la masa = 1/masa
    this.gravity_factor = 1;            // como reacciona a la gravedad (x defecto 1)

    // collission response 
    this.staticFriction = 0.5;          // friccion estatica
    this.dynamicFriction = 0.3;         // friccion dinámica
    this.restitution = 0.5;				// coeficiente de restitucion
    this.maxForce = 0;                  // maxima fuerza que resiste
    this.maxImpulse = 0;                 // 
	this.collider_list = null;			// lista de objetos con los que puede colisionar, null = todos

    // Varios
    this.name = "body";
    this.color = 'rgba(192,255,192,255)';
    this.id = 1;
    this.deleted = false;               // marcado para borrar
    this.img = null;
    this.visible = true;
		
}


//-------------------------------------------------------------------------------
// Interface para la creacion de cuerpos rigidos
//-------------------------------------------------------------------------------
// Facilitan la creacion de cuerpos rigidos tipicos como 
// circulos
// triangulos
// cajas
// poligonos 

/**
 * Interface para la creacion de cuerpos rigidos: Crea un cuerpo a partir de un objeto shape
 * @method Create
 * @param {Shape} p_shape forma geometrica
 * @param {float} p_density densidad
*/
Body.prototype.Create = function(p_shape,p_density) 
{
    // interface con la geometria
    this.density = p_density;
    this.shape = p_shape;
    p_shape.ComputeMass(p_density, this);
    p_shape.ComputeSize(this);
}

/**
 * Interface para la creacion de cuerpos rigidos: Crea un circulo
 * @method CreateCircle
 * @param {float} r radio
 * @param {Vector2} pos posicion
 * @param {float} p_density densidad
*/
Body.prototype.CreateCircle = function(r,pos, p_density)
{
    this.position = pos;
    this.Create(new GeoCircle(r) , p_density);
}

/**
 * Interface para la creacion de cuerpos rigidos: Crea una caja rectangular
 * @method CreateBox
 * @param {float} w width
 * @param {float} h height
 * @param {Vector2} pos posicion
 * @param {float} p_density densidad
*/
Body.prototype.CreateBox = function(w,h,pos, p_density)
{
    this.position = pos;
    var box_shape = new GeoPolygon();
    box_shape.CreateBox(w, h);
    this.Create(box_shape , p_density);
}

/**
 * Interface para la creacion de cuerpos rigidos: Crea un triangulo
 * @method CreateTri
 * @param {float} ax coordenada x punto a
 * @param {float} ay coordenada y punto a
 * @param {float} bx coordenada x punto b
 * @param {float} by coordenada y punto b
 * @param {float} cx coordenada x punto c
 * @param {float} cy coordenada y punto c
 * @param {float} p_density densidad
*/
Body.prototype.CreateTri = function(ax, ay , bx, by ,cx, cy, p_density)
{
    this.position.Set(0,0);
    var tri_shape = new GeoPolygon();
    var p1 = new Vector2(ax, ay);
    var p2 = new Vector2(bx, by);
    var p3 = new Vector2(cx, cy);
    tri_shape.CreateTri(p1,p2,p3);
    this.Create(tri_shape, p_density);
}


/**
 * Interface para la creacion de cuerpos rigidos: Crea un poligono convexo a partir de una nube de puntos
 * generando un convex hull.
 * El solver de collisiones solo soporta poligonos convexos, (limitacion del algoritmo SAT)
 * Ademas, por la convencion para computar normales, tiene que estar dibujado en sentido horario.
 * Como los puntos pueden ser una nube de puntos cualquieras, usualmente generado al azar, 
 * para asegurarse que el poligono es convexo, directamente, partimos de la nube de puntos y creamos un convex hull
 * De eso se encarga la funcion CreatePoly
 * @method CreatePoly
 * @param {Vector2[]} pt Array de puntos
 * @param {int} cant_pt cantidad de puntos
 * @param {float} p_density densidad
*/

Body.prototype.CreatePoly= function(pt,cant_pt, p_density)
{
    // genero un convex hull en sentido horario, a partir de la nube de puntos
    var vertices = [];
    var vertex_count = convexHull(pt, cant_pt, vertices);

    // creo el shape poligono, ya asegurado que es convexo y en sentido horario
    var poly_shape = new GeoPolygon();
    poly_shape.CreatePoly(vertices, vertex_count);
    this.Create(poly_shape, p_density);
	
	// muevo la posicion del poligono a su centro de gravedad
    this.position.Set(poly_shape.cm.x, poly_shape.cm.y);
}


//-------------------------------------------------------------------------------
// Dinamica del cuerpo rigido
//-------------------------------------------------------------------------------

/**
 * Aplica una fuerza al cuerpo rigido
 * el cuerpo rigido almacena la resulante de todas las fuerzas que actuan sobre el
 * @method ApplyForce
 * @param {Vector2} f fuerza 
*/
Body.prototype.ApplyForce = function (f)
{
    this.force.add(f);
}

/**
<P> Aplica una impulso al cuerpo rigido</P> 
<P> Un impulso es una fuerza que produce un cambio instantaneo en la velocidad. 
	Matematicamente es una fuerza de gran intensidad aplicada durante  un periodo corto de tiempo. </P> 

<P> http:*es.wikipedia.org/wiki/Delta_de_Dirac </P> 

<P> Desde el punto de vista de la dinamica del cuerpo rigido, para nosotros el impulso es un cambio instantaneo en el vector
 de velocidad, (es decir directamente cambiamos la velocidad sin tener que pasar por la aceleracion)
 En la vida real los cambios de velocidad se dan apartir de una aceleracion, producto de aplicar una fuerza, segun
 la segunda ley de movimiento de Newton</P>
 
<P>  F = ma </P> 

<P>  Sin embargo, el tiempo de aplicacion de la fuerza es menor que el elapsed_time de la simulacion, con lo cual 
 numericamente, es preferible modificar directamente la velocidad.
 El impulso produce un cambio en la velocidad (traslacion) del cuerpo rigido, que es inversamente proporcional a la masa. </P> 
 
<P> -------------------------------------------------------------</P> 
<P> ecuacion (1)                 F = ma</P>
<P> -------------------------------------------------------------</P> 

<P>  Si el impulso se produce en alguna linea de accion que pasa por el centro de masas (CM), solo se produce una movimiento 
 de traslacion. Pero en cualquier otro eje de acción, la fuerza tiene un cierto "momento" 
 El momento de la fuerza con respecto al eje, es el responsable de un cambio en el movimiento de rotacion. </P> 
 
<P>  http:*es.wikipedia.org/wiki/Momento_de_fuerza</P> 

<P> En  terminos simples el momento de la fuerza es tanto mayor cuanto mas alejado ese del eje que pasa sobre el CM
 Por ejemplo, si queremos abrir una puerta aplicamos una fuerza sobre la manija. Si quisieramos abrir la puerta aplicando
 esa misma fuerza sobre el centro de la fuerza, nos costaria mucho mas.</P>
 
<P> Por ultimo si quisieramos abrir la puerta aplicando la fuerza sobre la bisagra, no podriamos conseguir abrir la puerta
 La fuerza aplicada sobre el eje que pasa por el CM no tiene momento (el momento es nulo). 
Si el momento de la fuerza es cero, la fuerza no produce rotacion.</P>

<P> Para computar el momento de la fuerza, tambien llamado torque, se usa el producto vectorial del vector posicion del
 punto donde se aplica la fuerza,por el vector fuerza </P>

 <P>  T = r x F        T = torque, r = vector posicion, F = fuerza</P> 
 

 <P> En nuestro caso el punto de aplicacion es contactPoint y el vector fuerza es impuse </P>
<P>  Torque = cross_vec(contactPoint, impulse)</P> 

<P>  En 2 dimensiones, el producto cruz no es tan "intuitivo" como en 3d dimensiones. Es decir, no hay una forma 
 standard de computar un cross product en 2d. 
 En nuestro caso, lo que se precisa es la magnitud de lo que seria el cross product 3d si la coordenada Z fuerse cero
 Esa magnitud es la que va a producir un cambio en al velocidad de rotacion, o velocidad angular. </P> 

<P>  Ahora bien, no todos los torques producen la misma rotacion en los cuerpos rigidos, dependen de la resistencia que tenga
 el cuerpo rigido de rotar sobre el eje particular. 
 Ese concepto se llama momento de inercia, y es funciona de la misma forma que la masa para la aceleracion lineal.</P> 

<P>  Cuando queremos mover un objeto con mas masa tenemos que aplicar mas fuerza. La masa se puede ver como la resistencia
 que impone el cuerpo al movimiento lineal. Y como siempre esta descripta por la ecuacion de newton F = ma </P> 

<P>  De la misma forma, cuando hablamos de movimientos de rotacion, el analogo a la fuerza es el "momento de la fuerza", o torque
 y el analogo de la masa es el momento de inercia. El momento de inercia = I, depende no solo de la masa, (cuerpos mas masivos
 son mas dificiles de hacer girar que cuerpos mas livianos), si no tambien ademas de la distribuicion de la masa
 con respecto al eje sobre el que se quiere hacer girar. </P> 
 
<P>  http://es.wikipedia.org/wiki/Momento_de_inercia </P> 

<P> La ecuacion que describe estos 2 conceptos es </P>
<P>  -------------------------------------------------------------</P> 
<P> ecuacion (2)</P>
<P> T = I.dw/dt          T = torque, I = momento de inercia, dw/dt = aceleracion angular</P>
<P> -------------------------------------------------------------</P>
<P> http://es.wikipedia.org/wiki/Aceleraci%C3%B3n_angular</P>
<P> -------------------------------------------------------------</P>
 * @method ApplyImpulse
 * @param {Vector2} impulse impulso
 * @param {Vector2} contactPoint punto de contacto
*/
Body.prototype.ApplyImpulse = function (impulse, contactPoint)
{
    if (this.inverseMass == 0) {
        if (this.mass == 0)
            return;     // es un cuerpo estatico
            /*
        else {
            // si inverseMass = 0 pero mass >0 es porque era un cuerpo dinamico, que momentaneamente estaba 
            // en equilibrio ( se comportaba como un cuerpo estatico)
            // eso era valido mientras la unica fuerza que actuaba sobre el cuerpo es la gravedad
            // Si le aplicamos un impulso, es porque hay otra fuerza, se rompio el equilibrio, y se vuelve a comportar como dinamico
            this.inverseMass = 1 / this.mass;
        }*/
    }


    var mJ = impulse.length();

    if (mJ > this.maxImpulse)
        this.maxImpulse = mJ;

    if (this.inverseMass != 0 && this.maxForce != 0 && mJ > this.maxForce)
    {
        // el cuerpo revento, lo saco de la simulacion
        this.deleted = true;
        return;
    }


    // ecuacion (1)
    // F = ma
    this.velocity.add(vec2_multiply(impulse, this.inverseMass));

    // ecuacion (2)
    // T = I.dw/dt          T = torque, I = momento de inercia, dw/dt = aceleracion angular
    this.angularVelocity += this.inverseInertia * cross_vec(contactPoint, impulse);

}

/**
 * pone en cero el inverso de la masa indicando que el cuerpo está estático.
 * @method SetStatic
*/

// al poner en cero el inverso de la masa, se comporta como si el objeto tuviese masa infinita
// o sea que es estatico para los efectos de la simulacion
Body.prototype.SetStatic = function()
{
    this.inertia = 0;
    this.inverseInertia  = 0;
    this.mass = 0;
    this.inverseMass = 0;

}


/**
<P> lo pone en equilibrio estable, esto quiere decir que mientras no actue ninguna fuerza externa que no sea 
 la gravedad, el cuerpo se comporta como un objeto estatico, es decir con masa infinita
 En la practica lo unico que hacemos es poner el inverso de la masa en cero, para que no acelere mas
 Sin embargo a diferfencia de los objetos estaticos, la masa del cuerpo sigue siendo positiva (y no cero = infinito)
 de esta forma, se distingue entre un objeto estatico pp dicho, de otro que esta en equilibrio estable por el momento.</P>
 * @method SetStaticEquilibrium
*/
Body.prototype.SetStaticEquilibrium = function () {
    this.inverseMass = 0;
}


/**
 * Para el caso 2d la orientacion es un ángulo, representa el angulo de giro sobre el eje Z que pasa por el CM
 * @method SetOrient
 * @param {float} angle angulo de giro en radianes
*/
Body.prototype.SetOrient = function( angle)
{
    this.orient = angle;
    this.shape.SetOrient( angle );
}



/**
 <P> independientemente que las clases geometrias (circulos y poligonos) puedan computar la masa y el momento de inercia
	a partir de la definicion usando conceptos geometricos. Muchas veces en el contexto de una simulacion, el cuerpo rigido
	es una simplificacion de un cuerpo mas complicado, y es preferible "tunear" a mano los valores de masa e inercia como 
	si fueran un parametro de diseño</P>
 * @method SetMass
 * @param {float} m masa
*/
Body.prototype.SetMass = function (m)
{
    this.mass = m;
    this.inverseMass = m!=0 ? 1/m: 0;
}

/**
 * @method SetInertia
 * @param {float} I intercia
*/
Body.prototype.SetInertia = function( I)
{
    this.inertia = I;
    this.inverseInertia = I!=0 ? this.inverseInertia = 1/I: 0;

}

/**
<P> Ecuaciones de newton que tenemos que integrar para computar la posicion del cuerpo rigido</P>
<P>  Acceleration</P>
<P>     F = mA</P>
<P>  => A = F * 1/m</P>
<P>  Explicit Euler</P>
<P>  x += v * dt</P>
<P>  v += (1/m * F) * dt</P>
<P>  Symplectic Euler</P>
<P>  v += (1/m * F) * dt</P>
<P>  x += v * dt</P>
<P>  Para integrar conviene hacerlo en 2 etapas, avanzando el tiempo por la mitad. La explicacion se puede encontrar aca</P>
<P>  http://www.niksula.hut.fi/~hkankaan/Homepages/gravity.html</P>
<P>  se supone que dt = elapsed_time / 2</P>
 * @method IntegrateForces
 * @param {float} dt elapsed time / 2
*/
Body.prototype.IntegrateForces = function( dt)
{
    if(this.inverseMass == 0)
        return;

    //	velocity += (force * inverseMass + gravity*gravity_factor) * dt;

    // aceleracion que producen las fuerzas externas
    var acel = vec2_multiply(this.force, this.inverseMass);
    // aceleracion de la gravedad
    var acel_grav = vec2_multiply(gravity, this.gravity_factor);
    // aceleracion neta
    var acel_neta = vec2_add(acel, acel_grav);
    // integro la aceleracion lineal
    this.velocity.add(vec2_multiply(acel_neta, dt));
    // integro la aceleracion angular
    this.angularVelocity += this.torque * this.inverseInertia * dt;

}


/**
 * @method IntegrateVelocity
 * @param {float} dt elapsed time / 2
*/
Body.prototype.IntegrateVelocity = function(dt)
{
    if(this.inverseMass == 0)
        return;

    this.position.add( vec2_multiply(this.velocity ,  dt));
    this.orient += this.angularVelocity * dt;
    this.orient = fmod(this.orient, 2.0 * Math.PI);
    this.SetOrient(this.orient);

}

/**
 * @method Render
*/
Body.prototype.Render = function()
{

    if (!this.visible)
        return;

    if (this.img != null) {
        // tiene una imagen asociada, dibuja atraves de la imagen

        var x = ox + this.position.x*ex;
        var y = oy + this.position.y*ey;
        var width = this.width * ex;
        var height = this.height * ey;

        ctx.translate(x, y);
        ctx.rotate(this.orient);
        ctx.drawImage(this.img, -width / 2, -height / 2, width, height);
        ctx.rotate(-this.orient);
        ctx.translate(-x, -y);
    }
    else {
        // dibujo a traves de la interface de geometria asociada al cuerpo
        this.shape.Render(this);
    }

}

/**
 * helper, dibuja un centro de gravedad en la posicion dada
 * @method drawCenterOfMass
 * @param {float} x posicion x
 * @param {float} y posicion y
 * @param {float} r radio
 * @param {float} alfa angulo en radianes
*/
 function drawCenterOfMass(x,y,r,alfa) 
{
    var da = Math.PI / 2;

    for (var i = 0; i < 4; ++i)
    {
        ctx.fillStyle = (i%2)==0 ? '#ffffff' : '#000000';
        ctx.beginPath();
        ctx.moveTo(ox + x * ex, oy + y * ey);
        ctx.lineTo(ox + (x + r * Math.cos(alfa)) * ex, oy + ( y + r * Math.sin(alfa)) * ey);
        ctx.arc(ox + x * ex, oy + y * ey, r * ex, alfa, alfa + da);
        ctx.closePath();
        ctx.fill();
        alfa += da;
    }
}
