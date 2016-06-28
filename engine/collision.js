

/**
 * @fileoverview collision.js
<P> Implementa colisiones entre 2 cuerpos</P>
<P>  Fase de Collision Detection</P> 
<P> Los motores de fisicas usualmente tienen 2 etapas bien definidas</P>
<P> * Etapa de Collision Detection: el objetivo de esta etapa es determinar si 2 o mas cuerpos entran en colision. </P>
<P> PAra ello se aplican una serie de algoritmos geometricos bastante estudiados. </P>
<P> Esta etapa a su vez se puede dividir en 2 fases :</P>
<P>              - Broad Phase. Se trata de rapidamente descartar aquellos pares de cuerpos que por estar lo suficienemente
                              alejados no pueden entrar en colision. Se suelen usar bounding box y otro tipo de simplificaciones
                              y distintas estructuras jerarquicas. La idea es que rapidamente se puedan detectar pares de 
                              candidatos que pueden entrar en colision.</P>
							  
<P>              - Narrow Phase. Con el resultado de pares "posibles" de la fase anterior, se determinar exactamente si 
                              esos "candidatos" a colision, realmente estaban colisionando, y en caso afirmativo
                              colectar informacion especifica de la colision, que luego va a servir en etapas posteriores
                              como el punto exacto de la colision, la normal, y la penetracion.</P>
							  
<P> * Etapa de Collision Response : una vez dectectada la collision, el motor tiene que tomar alguna acción para evitar
 que los cuerpos que collisionaron siguen en esa situacion. En nuestro caso se resuelve aplicando un impulso de tal forma
 que los cuerpos se alejen en la direccion que chocaron,  y asi evitar que sigan en colision = Impulse Resolution.</P>
 

<P> En este motor no tenemos Broad Phase, los algoritmos que se codifican a continuacion, corresponden a la llamada
 Narrow Phase, y tienen por objetivo determianr si hay o no colision, y en casi que asi fuera computar:</P>
 
<P> i) punto de contacto</P>
<P> ii) direccion normal de la colision</P>
<P> iii) penetracion</P>
<P> puede que los cuerpos entre en colision en mas de un punto, por eso la informacion de contacto tiene un array 
y una cantidad de contactos.</P>

*/

// -------------------------------------------------------------
//	HELPERS para Deteccion de colisiones
// -------------------------------------------------------------

/**
 * Colision entre esferas
 * @function CircleCircle
 * @param {Body} a esfera A 
 * @param {Body} b esfera B 
 * @param {ContactSolver} contact_info informacion de contacto
 */
function CircleCircle(a, b ,contact_info)
{
    var A = a.shape;            
    var B = b.shape;

    var normal = vec2_substract(b.position , a.position);
    var dist_sqr = normal.lengthSqr();
    var radius = A.radius + B.radius;

    // No estan en contacto
    if(dist_sqr >= radius * radius)
    {
        contact_info.contact_count = 0;
        return false;
    }

    var distance = Math.sqrt(dist_sqr);
    contact_info.contact_count = 1;

    if(Math.abs(distance) < EPSILON)
    {
        // Estan tocando justo
        contact_info.penetration = A.radius;
        contact_info.normal = new Vector2( 1, 0 );
        contact_info.contacts[0] = a.position;
    }
    else
    {
        contact_info.penetration = radius - distance;
        contact_info.normal = normal.multiply(1/distance);
        contact_info.contacts[0] = vec2_add(vec2_multiply(normal, A.radius), a.position);
    }
    return true;
}


// Para detectar colisiones entre polygonos convexos (y entre esferas con poligonos) se usa SAT
// SAT = Separation Axis teorem = SAT 
// La idea es que si 2 convexos NO se estan tocando, entonces existe un eje sobre el cual, al proyectar todos los 
// puntos, existe una separacion. 


/**
 * Colision entre esfera y Poligono
 * Para esfera contra poligono se usa una adaptacion del SAT
 * @function CirclePolygon
 * @param {Body} a esfera a 
 * @param {Body} b poligono b 
 * @param {ContactSolver} contact_info informacion de contacto
 */
function CirclePolygon(a, b, contact_info)
{
    var A = a.shape;            // circulo
    var B = b.shape;            // poligono

    contact_info.contact_count = 0;

    // Transoformo todo a al espacio del poligono
    // Para transformar un vector del world space al polygon B space, tengo que restarle la posicion del CM del poligono B
    // Y luego multiplicarlo por la matriz de transformacion inversa del B (que es igual a la transpuesta, por ser una matriz
    // de rotacion).

    // Demostracion
    // Los puntos en model space (o poligon space) se transforman a world space asi:
    // (subindice ws = world space)
    // Pws = B.matWorld*P + B.pos
    // luego si estoy al reves, y tengo Pws y quiero P tengo que despejar
    // P = inversa(B.matWorld) *(Pws - B.pos)
    // como es una matriz de rotacion, su inversa es igual a su transpuesta
    // P = transpuesta(B.matWorld) *(Pws - B.pos)

    var center = vec2_transform(B.matWorld.Transpose()  , vec2_substract(a.position , b.position));
	
    // Calcular el lado donde se produce la minima penetracion
    var separation = -1000000;
    var faceNormal = 0;
    for(var i = 0; i < B.m_vertexCount; ++i)
    {
		var s = dot( B.normals[i], vec2_substract(center , B.vertices[i]));
		if(s > A.radius)
			return false;

		if(s > separation)
        {
			separation = s;
			faceNormal = i;
        }
    }

    // Adaptacion de algoritmo SAT 
    var v1 = B.vertices[faceNormal];
    var v2 = B.vertices[faceNormal+1];

    // Verifico si el el punto center esta dentro del poligono
    if(separation < EPSILON)
    {
        // El punto esta dentro del poligono
        contact_info.contact_count = 1;
        contact_info.normal = vec2_multiply(vec2_transform(B.matWorld , B.normals[faceNormal]) , -1);
        contact_info.contacts[0] = vec2_add(vec2_multiply(contact_info.normal , A.radius ) , a.position);
        contact_info.penetration = A.radius;
        return true;
    }

    var dot1 = dot( vec2_substract(center , v1), vec2_substract(v2, v1));
    var dot2 = dot( vec2_substract(center , v2), vec2_substract(v1 ,v2));
    contact_info.penetration = A.radius - separation;

    // Esta cerca del punto V1
    if(dot1 <= 0.)
    {
        
        if(vec2_substract(center,v1).lengthSqr() > A.radius * A.radius)
            return false;

        contact_info.contact_count = 1;
        var n = vec2_substract(v1 , center);
        n = vec2_transform(B.matWorld , n);
        n.normalize();
        contact_info.normal = n;
        v1 = vec2_add(vec2_transform(B.matWorld ,v1) , b.position);
        contact_info.contacts[0] = v1;
    }

    // Esta cerca del punto V2
    else if (dot2 <= 0.)
    {
        if(vec2_substract(center,v2).lengthSqr() > A.radius * A.radius)
            return false;

        contact_info.contact_count = 1;
        var n = vec2_substract(v2, center);
        v2 = vec2_add(vec2_transform(B.matWorld ,v2) , b.position);
        contact_info.contacts[0] = v2;
        n = vec2_transform(B.matWorld ,n);
        n.normalize( );
        contact_info.normal = n;
    }
    else
    // en un punto medio entre V1 y V2
    {
        var n = B.normals[faceNormal];
        if(dot( vec2_substract(center ,v1), n ) > A.radius)
            return false;

        n = vec2_transform(B.matWorld , n);
        contact_info.normal = vec2_multiply(n,-1);
        contact_info.contacts[0] = vec2_add(vec2_multiply(contact_info.normal ,  A.radius) , a.position);
        contact_info.contact_count = 1;
    }

    return true;
}

// 2D Polygon Collision Detection
// http://www.codeproject.com/Articles/15573/D-Polygon-Collision-Detection
// http://en.wikipedia.org/wiki/Hyperplane_separation_theorem
// http://back2basic.phatcode.net/?Issue_%231:2D_Convex_Polygon_Collision_using_SAT
function getSeparationAxis( faceIndex, a, b)
{
    A = a.shape;
    B = b.shape;
    var max = -100000;

    for(var i = 0; i < A.m_vertexCount; ++i)
    {
        // Retrieve a face normal from A
		var n = A.normals[i];
		var nw = vec2_transform(A.matWorld ,  n);

        // Transformo al espacio local de B
		var bT = B.matWorld.Transpose();
		n = vec2_transform(bT ,  nw);

        // maxima distancia de B sobre la direccion -n
		var s = B.getMaxProjection(vec2_multiply(n, -1));

        // Transformo al espacio local de B
		var v = A.vertices[i];
		v = vec2_add(vec2_transform(A.matWorld , v) , a.position);
		v.substract(b.position);
		v = vec2_transform(bT , v);

        // distancia de penetration distance
		var d = dot( n, vec2_substract(s , v) );

		if (d > max)
        {
		    max = d;
		    faceIndex.x = i;
        }
    }
    return max;
}

// nomenclatura:
// Inicidente = es el poligono que esta chocando = IncBody
// Reference = es el poligono chocado = RefBody
// Es una cuestion de puntos de vista, se dice inicidente y referente, porque se transforma todas las coordenadas
// al espsacio del referente, y en ese espacio, el poligono esta "quieto", Luego la velocidad del incidente se computa
// de acuerdo a la velocidad relativa en ese marco de referencia. 

function getIncFace( v, RefBody, IncBody, refIndex )
{
	var RefPoly = RefBody.shape;
    var IncPoly = IncBody.shape;
	
    var refNormal = RefPoly.normals[refIndex];

    // Calculate normal in incident's frame of reference
    refNormal = vec2_transform(RefPoly.matWorld , refNormal);               // To world space
    refNormal = vec2_transform(IncPoly.matWorld.Transpose( ) , refNormal);  // To incident's model space

    // Find most anti-normal face on incident polygon
    var incFace = 0;
    var minDot = 100000;
    for(var i = 0; i < IncPoly.m_vertexCount; ++i)
    {
        var d = dot( refNormal, IncPoly.normals[i]);
        if(d < minDot)
        {
            minDot = d;
            incFace = i;
        }
    }

    // Almaceno los puntos de contacto
    v[0] = vec2_add(vec2_transform(IncPoly.matWorld , IncPoly.vertices[incFace]) , IncBody.position);
    incFace = incFace + 1 >= IncPoly.m_vertexCount ? 0 : incFace + 1;
    v[1] = vec2_add(vec2_transform(IncPoly.matWorld , IncPoly.vertices[incFace]) , IncBody.position);
}

function clipFace( n, c, face)
{
    var sp = 0;
    var out = [face[0].Clone(), face[1].Clone()];

    // d = ax + by - c
    var d1 = dot( n, face[0] ) - c;
    var d2 = dot( n, face[1] ) - c;

    // estan dentras del plano? 
    if (d1 <= 0)
        out[sp++] = face[0].Clone();

    if (d2 <= 0)
        out[sp++] = face[1].Clone();

    // estan en semiplanos diferentes ? 
    if(d1 * d2 < 0)
    {
        // meto el punto de interseccion
        var alpha = d1 / (d1 - d2);
        out[sp] = vec2_add(face[0] , vec2_multiply(vec2_substract(face[1] , face[0]) , alpha));
        ++sp;
    }

    face[0] = out[0];
    face[1] = out[1];
    return sp;
}

/**
 * Colision entre dos Poligono (convexos)
 * implementa el algoritmo SAT
 * @function PolygonPolygon
 * @param {Body} a poligono a 
 * @param {Body} b poligono b 
 * @param {ContactSolver} contact_info informacion de contacto
 */
function PolygonPolygon( a, b ,contact_info)
{
	var A = a.shape;
	var B = b.shape;

	contact_info.contact_count = 0;

	if (b.position.y > 45)
	    var vp = 1;

    // Busco un "separating axis" en las caras del poligono A
	var faceA = new Vector2(0,0);           // uso solo la x, pero no tenia forma de pasar por referencia un valor int
	var penetrationA = getSeparationAxis(faceA, a, b);
    // 
	if (penetrationA > EPSILON)
		return false;

    // Busco un "separating axis" en las caras del poligono B
	var faceB = new Vector2(0, 0);
	var penetrationB = getSeparationAxis( faceB, b, a );
	if(penetrationB > EPSILON)
		return false;

	var refIndex;
	var flip;       // Always point from a to b
	var RefBody;    // Reference
	var IncBody;    // Incident

    // Cual de los 2 poligonos contiene el "reference face" refFace
	var k_biasRelative = 0.95;
	var k_biasAbsolute = 0.01;
	if (penetrationA >= penetrationB * k_biasRelative + penetrationA * k_biasAbsolute)
    {
		RefBody = a;
		IncBody = b;
		refIndex = faceA.x;
		flip = false;
    }
    else
    {
		RefBody = b;
		IncBody = a;
		refIndex = faceB.x;
		flip = true;
    }
    
	RefPoly = RefBody.shape;        // Reference
	IncPoly = IncBody.shape;        // Incident

	var incFace = [new Vector2(0,0) , new Vector2(0,0)];
	getIncFace( incFace, RefBody, IncBody, refIndex);
	var v1 = RefPoly.vertices[refIndex];
	var v2 = RefPoly.vertices[refIndex+1];

    // Transformo a world psace
	v1 = vec2_add(vec2_transform(RefPoly.matWorld , v1) , RefBody.position);
	v2 = vec2_add(vec2_transform(RefPoly.matWorld , v2) , RefBody.position);
    // computo el reference 
	var sidePlaneNormal = vec2_substract(v2 , v1);
	sidePlaneNormal.normalize( );

	var refFaceNormal = new Vector2(sidePlaneNormal.y, -sidePlaneNormal.x);

    // ecuacion de la recta
    // ax + by = c
    // c es la distancia al origen
	var refC = dot( refFaceNormal, v1 );
	var negSide = -dot( sidePlaneNormal, v1 );
	var posSide =  dot( sidePlaneNormal, v2 );

    // determino si el punto esta dentro de los limites de la arista (clipping)
	if (clipFace(vec2_multiply(sidePlaneNormal, -1), negSide, incFace) < 2)
		return false; 

	if(clipFace(  sidePlaneNormal, posSide, incFace ) < 2)
		return false; 

    // Flip= indica que quedo al reves, hay que usar -dir
	contact_info.normal = flip ? vec2_multiply(refFaceNormal,-1) : refFaceNormal;

    // computo los puntos de contacto pp dichos
	var cp = 0; 
	var separation = dot( refFaceNormal, incFace[0] ) - refC;
	if(separation <= 0)
    {
		contact_info.contacts[cp] = incFace[0];
		contact_info.penetration = -separation;
		++cp;
    }
    else
		contact_info.penetration = 0;

	separation = dot( refFaceNormal, incFace[1] ) - refC;
	if(separation <= 0)
    {
		contact_info.contacts[cp] = incFace[1];
		contact_info.penetration += -separation;
		++cp;

        // si hay mas 2 un punto de contacto, promedio la penetracion
		contact_info.penetration /= cp;
    }

	contact_info.contact_count = cp;
	return true;
}


