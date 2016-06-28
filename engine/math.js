/**
 * @fileoverview math.js
	Implementa algebra lineal de vectores 2d 
	matrices 2 x 2, usadas para la rotacion del cuerpo rigido
 */


var EPSILON = 0.0001;

/**
 * Vector2
 * @class vector de 2 dimensiones.
 * @constructor
 * @param {float} x - componente x
 * @param {float} y - componente y
 */

function Vector2(x, y) {
    this.x = x;
    this.y = y;
};

/**
* Copia el contenido de un vector
* @method copyFrom
* @return {Vector2} this
*/
Vector2.prototype.copyFrom = function (v) {
    this.x = v.x;
    this.y = v.y;
};


/**
* Clona un vector
* @method Clone
* @return la copia del vector (vector clonado). Es un nuevo objeto. 
*/

Vector2.prototype.Clone = function () {
    return new Vector2(this.x, this.y);
};


/**
 * Asigna los componentes x,y del vector
 * @method Set
 * @param {float} x - componente x
 * @param {float} y - componente y
 */

Vector2.prototype.Set = function (x,y) 
{
    this.x = x;
    this.y = y;
};

/**
 * Suma vectorial unitaria:  this += v
 * @method add
 * @param {Vector2} v
 * @return {Vector2} this
 */
Vector2.prototype.add = function (v) {
    this.x += v.x;
    this.y += v.y;
    return this;
};

/**
 * Multiplicacion por escalar unitaria:  this *= k
 * @method multiply
 * @param {float} k
 * @return {Vector2} this
 */
Vector2.prototype.multiply = function (k) {
    this.x *= k;
    this.y *= k;
    return this;
};

/**
 * resta vectorial unitaria:  this *= k
 * @method substract
 * @param {Vector2} v
 * @return {Vector2} this
 */
Vector2.prototype.substract = function (v) {
    this.x -= v.x;
    this.y -= v.y;
    return this;
};

/**
 * modulo al cuadrado
 * @method lengthSqr
 * @return {float} modulo del vector elevado al cuadrado
 */
Vector2.prototype.lengthSqr = function () {
    return this.x * this.x + this.y * this.y;
};

/**
 * modulo 
 * @method length
 * @return {float} modulo del vector
 */
Vector2.prototype.length = function () {
    return Math.sqrt(this.lengthSqr());
};


/**
 * Normaliza el vector.
 * @method normalize
 * @return {Vector2} this normalizado
 */
Vector2.prototype.normalize = function () {
    var len = this.length();
    if (len >= EPSILON) {
        this.x /= len;
        this.y /= len;
    }
    return this;
};


/**
 * rota con respecto al origen (0,0)
 * @method rotate
 * @param {float} an - ángulo en radianes
 * @return {Vector2} this rotado
 */
Vector2.prototype.rotate = function (an) {
    var cosa = Math.cos(an);
    var sina = Math.sin(an);
    var xp = this.x * cosa - this.y * sina;
    var yp = this.x * sina + this.y * cosa;
    this.x = xp;
    this.y = yp;
    return this;
};


// --------------------------------------------
// funciones globales
// --------------------------------------------

/**
 * Suma vectorial binaria:  rta = u + v. Retorna un nuevo objeto Vector2
 * @function vec2_add
 * @param {Vector2} u
 * @param {Vector2} v
 * @return {Vector2} u + v 
 */
function vec2_add(u, v) {
    return new Vector2(u.x + v.x, u.y + v.y);
};

/**
 * resta vectorial binaria:  rta = u - v. Retorna un nuevo objeto Vector2
 * @function vec2_add
 * @param {Vector2} u
 * @param {Vector2} v
 * @return {Vector2} u - v 
 */
function vec2_substract(u, v) {
    return new Vector2(u.x - v.x, u.y - v.y);
};

/**
 * producto entre vector y escalar.  rta = u *k. Retorna un nuevo objeto Vector2
 * @function vec2_multiply
 * @param {Vector2} u
 * @param {float} k
 * @return {Vector2} u*k
 */
function vec2_multiply(u, k) {
    return new Vector2(u.x * k, u.y * k);
};

/**
 * vector normal (a 90 grados). Retorna un nuevo objeto Vector2
 * @function vec2_normal
 * @param {Vector2} u
 * @return {Vector2} vector normal a u
 */
function vec2_normal(u) {
    return new Vector2(-u.y, u.x);
};


/**
 * producto escalar (dot product)
 * @function dot
 * @param {Vector2} u
 * @param {Vector2} v
 * @return {float} u.v
 */
function dot(u, v) {
    return u.x * v.x + u.y * v.y;
};

/**
 * producto vectorial 2d (cross product)
 * version 1:  escalar x vector = vector normal a v, y escalado por k
 * @function cross
 * @param {float} k
 * @param {Vector2} v
 * @return {Vector2} vector normal a v, y escalado por k
 */
function cross(k, v) {
    return new Vector2(-v.y * k, v.x * k);
};


/**
 * producto vectorial 2d (cross product)
 * version 2:  vector x vector = escalar, devuelve la magnitud del vector equivalente al cross 3d standard
 * @function cross_vec
 * @param {Vector2} a
 * @param {Vector2} b
 * @return {float} magnitud del vector equivalente al cross 3d standard
 */
function cross_vec(a, b) {
    return a.x * b.y - a.y * b.x;
};


/**
 * Matrix2
 * @class Matriz de 2 x 2 
 * @constructor
 * @param {float} a - componente a11
 * @param {float} b - componente a12
 * @param {float} c - componente a21
 * @param {float} d - componente a22
 */
function Matrix2(a, b,c,d) {
    this.m00 = a;
    this.m01 = b;
    this.m10 = c;
    this.m11 = d;
};

/**
 * Matriz Identidad
 * @method Identity
 * @return {Matrix2} Matriz identidad (no crea un nuevo objeto)
 */
Matrix2.prototype.Identity = function () {
    this.m00 = 1;
    this.m01 = 0;
    this.m10 = 0;
    this.m11 = 1;
    return this;
};

/**
 * Matriz de Rotacion
 * @method Rotation
 * @param {float} angle -  angulo de rotación (sobre el eje Z)
 * @return {Matrix2} Matriz de rotación (no crea un nuevo objeto)
 */
Matrix2.prototype.Rotation = function (angle) {
    var c = Math.cos(angle);
    var s = Math.sin(angle);

    this.m00 = c;
    this.m01 = -s;
    this.m10 = s;
    this.m11 = c;
    return this;
};
	

/**
 * Matriz Transpuesta
 * @method Transpose
 * @return {Matrix2} Matriz de transpuesta. Crea un NUEVO objeto 
 */
Matrix2.prototype.Transpose = function () {
    return new Matrix2(this.m00, this.m10, this.m01, this.m11);
};

/**
 * Multiplicacion por un vector. vec_rta = M * vec
 * @method multiplyVec
 * @param {Vector2} rhs
 * @return {Vector2} this x rhs. Crea un NUEVO objeto 
 */
Matrix2.prototype.multiplyVec = function (rhs) {
    return new Vector2(this.m00 * rhs.x + this.m01 * rhs.y, this.m10 * rhs.x + this.m11 * rhs.y);
};
 
/**
 * Multiplicacion por una matriz. 
 * @method multiplyMat
 * @param {Matrix2} rhs
 * @return {Matrix2} this x rhs. Crea un NUEVO objeto 
 */
Matrix2.prototype.multiplyMat = function (rhs) {
    return new Matrix2(
		this.m00 * rhs.m00 + this.m01 * rhs.m10,
		this.m00 * rhs.m01 + this.m01 * rhs.m11,
		this.m10 * rhs.m00 + this.m11 * rhs.m10,
		this.m10 * rhs.m01 + this.m11 * rhs.m11
		);
};


/**
 * Transformacion x un vector
 * @function vec2_transform
 * @param {Matrix2} m
 * @param {Vector2} v
 * @return {Vector2} m x v. Crea un NUEVO objeto 
 */
function vec2_transform(m, v) {
    return new Vector2(m.m00 * v.x + m.m01 * v.y, m.m10 * v.x + m.m11 * v.y);
};


/**
 * Numero aleatorio
 * @function Random
 * @param {float} x0 - numero desde
 * @param {float} x1 - numero hasta
 * @return {float} número aleatorio entre x0 y x1
 */
// numero aleatorio
function Random(x0 , x1)
{
    return Math.random() * (x1-x0) + x0;
}


/**
 * Color aleatorio
 * @function getRandomColor
 * @return {string} color aleatorio
 */
function getRandomColor() {
    var letters = '0123456789ABCDEF'.split('');
    var color = '#';
    for (var i = 0; i < 6; i++) {
        color += letters[Math.floor(Math.random() * 16)];
    }
    return color;
}

// ------------------------------------------------------------------------------------------------

// algoritmo de convex hull, ademas re-ordena los puntos para que el poligono quede en sentido horario
// explicacion del algoritmo 
// http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
// basic math:
// See : http://www.oocities.org/pcgpe/math2d.html

function convexHull(vertices, count , vertices_out)
{
    // si tiene menos de 3 vertices, no hay poligono
    if (count < 3)
        return;

    // buscamos el maximo x
    var index_x_max = 0;
    var x_max = vertices[0].x;
    for (var i = 1; i < count; ++i) {
        var x = vertices[i].x;
        if (x > x_max) {
            x_max = x;
            index_x_max = i;
        }
        else
            // si son iguales, tiene en cuenta el valor y
            if (x == x_max)
                if (vertices[i].y < vertices[index_x_max].y)
                    index_x_max = i;
    }

    var hull = [];
    var outCount = 0;
    var indexHull = index_x_max;

    for (; ;) {
        hull[outCount] = indexHull;
        // Search for next index that wraps around the hull
        // by computing cross products to find the most counter-clockwise
        // vertex in the set, given the previos hull index
        var nextHullIndex = 0;
        for (var i = 1; i < count; ++i) {
            // Skip if same coordinate as we need three unique
            // points in the set to perform a cross product
            if (nextHullIndex == indexHull) {
                nextHullIndex = i;
                continue;
            }

            // Cross every set of three unique vertices
            // Record each counter clockwise third vertex and add
            // to the output hull
            var e1 = vec2_substract(vertices[nextHullIndex], vertices[hull[outCount]]);
            var e2 = vec2_substract(vertices[i], vertices[hull[outCount]]);
            var c = cross_vec(e1, e2);
            if (c < 0)
                nextHullIndex = i;

            // Cross product is zero then e vectors are on same line
            // therefor want to record vertex farthest along that line
            if (c == 0 && e2.lengthSqr() > e1.lengthSqr())
                nextHullIndex = i;
        }
        ++outCount;
        indexHull = nextHullIndex;

        // Conclude algorithm upon wrap-around
        if (nextHullIndex == index_x_max)
        {
            m_vertexCount = outCount;
            break;
        }
    }

    // Copy vertices into shape's vertices
    for (var i = 0; i < m_vertexCount; ++i)
        vertices_out[i] = vertices[hull[i]].Clone();

    return m_vertexCount;
}



function fmod(x, y) {
    //  discuss at: http://phpjs.org/functions/fmod/
    // original by: Onno Marsman
    //    input by: Brett Zamir (http://brett-zamir.me)
    // bugfixed by: Kevin van Zonneveld (http://kevin.vanzonneveld.net)
    //   example 1: fmod(5.7, 1.3);
    //   returns 1: 0.5

    var tmp, tmp2, p = 0,
      pY = 0,
      l = 0.0,
      l2 = 0.0;

    tmp = x.toExponential()
      .match(/^.\.?(.*)e(.+)$/);
    p = parseInt(tmp[2], 10) - (tmp[1] + '')
      .length;
    tmp = y.toExponential()
      .match(/^.\.?(.*)e(.+)$/);
    pY = parseInt(tmp[2], 10) - (tmp[1] + '')
      .length;

    if (pY > p) {
        p = pY;
    }

    tmp2 = (x % y);

    if (p < -100 || p > 20) {
        // toFixed will give an out of bound error so we fix it like this:
        l = Math.round(Math.log(tmp2) / Math.log(10));
        l2 = Math.pow(10, l);

        return (tmp2 / l2)
          .toFixed(l - p) * l2;
    } else {
        return parseFloat(tmp2.toFixed(-p));
    }
}