
/**
 * @fileoverview constraint.js
<P> Implementa restricciones entre 2 cuerpos</P>
<P>  -distancia</P>
<P>  -angulo</P>
<P>  -joint</P>
*/

/**
 * Constraint
 * @class Constraint Restricciones entre dos cuerpos A y B
 * @property {Body} A cuerpo A 
 * @property {Body} B cuerpo B 
 * @property {Vector2} ptA pivote o punto de contacto con el cuerpo A (si no esta definido usa el Centro de masas)
 * @property {Vector2} ptB pivote o punto de contacto con el cuerpo B (si no esta definido usa el Centro de masas)
 * @property {float} dist parametros del constraint: distancia
 * @property {float} angle parametros del constraint: angulo
 * @property {boolean} joint flag que indica si el constraint es del tipo articulacion
 * @property {boolean} exact_distance flag que indica si el constraint de distancia es exacto, o si la distancia es menor o igual
 * @property {boolean} angle_constraint flag que indica si el constraint del tipo restriccion de angulo en una articulacion
 * @property {boolean} visible flag que indica si el graficador tiene que dibujar el constraint
 * @constructor
 * @param {Body} A cuerpo A 
 * @param {Body} B cuerpo B 
 * @param {float} dist distancia
 */

function Constraint(A,B,dist)
{
    // cuerpos que intervienen
    this.A = A;
    this.B = B;
    // pivotes: x defecto el pto se toma en el centro de gravedad
    this.ptA = new Vector2(0,0);
    this.ptB = new Vector2(0,0);
    // parametros del constraint
    this.dist = dist;
    this.angle = 0;
    // propiedades
    this.joint = false;
    this.exact_distance = false;
    this.angle_constraint = false;
    this.visible = 1;
};

/**
 * Crea un constraint de distancia entre 2 cuerpos y en una posicion especifica (pivotes de contacto)
 * @function CreateDistanceConstraint
 * @param {Body} A cuerpo A 
 * @param {Vector2} ptA pivote o punto de contacto con el cuerpo A 
 * @param {Body} B cuerpo B 
 * @param {Vector2} ptB pivote o punto de contacto con el cuerpo B 
 */
function CreateDistanceConstraint(A, ptA, B, ptB) {
    var c = new Constraint(A, B, 0);
    c.ptA = ptA;
    c.ptB = ptB;
    var rA = vec2_transform(A.shape.matWorld, ptA);
    rA.add(A.position);
    var rB = vec2_transform(B.shape.matWorld, ptB);
    rB.add(B.position);
    var r = vec2_substract(rA, rB);
    c.dist = r.length();
    c.joint = false;
    c.angle_constraint = false;
    c.exact_distance = true;
    c.visible = 1;
    return c;
}


Constraint.prototype.Solve = function()
{
    // proyecto los puntos a world space y obtengo la distancia entre los pivotes
    var A = this.A;
    var B = this.B;

    var rA = vec2_transform(A.shape.matWorld , this.ptA);
    var rB = vec2_transform(B.shape.matWorld, this.ptB);

    // vector entre los puntos de contacto en world space
    // vr = B->position + rB - A->position - rA;
    var vr = vec2_substract(vec2_add(rB, B.position), vec2_add(rA, A.position));

    // verifico si cumple con el constraint
    var dreal = vr.length();
    if(this.exact_distance)
    {
        if(Math.abs(dreal-this.dist)<0.01)
            return;         // verifica el constraint, vuelvo
    }
    else
    {
        if(dreal<this.dist)
            return;         // verifica el constraint, vuelvo
    }
	
    // no se cumple la restriccion de distancia
    var n = vec2_multiply(vr , (1/dreal));		// direccion de A hacia B normalizada
    var D = dreal - this.dist;			// distancia que tengo que corregir 
    var dt = fixed_dt;			// en cierta cantidad de tiempo

    // Para ello genero una fuerza ficticia en direccion hacia el objeto B, que compense la velocidad relativa de A
    // y lo haga dirigirse hacia B 
    // tomo solo la parte de la velocidad de A sobre la linea de fuerza n
    // uso la velocidad relativa entre ambos cuerpos
    var velRel = vec2_substract(A.velocity , B.velocity);
    var j = D/dt -dot(n,velRel);
    j*=0.3;

    // distribuyo el impulso de forma proporcional a las masas de ambos cuerpos
    var t;
    if(A.inverseMass==0)
        t = 0;
    else
    if(B.inverseMass==0)
        t = 1;
    else
        t = A.mass / (A.mass + B.mass);

    var jA = j * t;
    var jB = j * (1-t);


    // Primero corrijo el Objeto A
    var nA = vec2_multiply(n,jA);
    if(A.inverseMass!=0)
        A.velocity.add(nA);
    if(A.inverseInertia!=0)
        A.SetOrient( A.orient + A.inverseInertia * cross_vec(rA,nA));


    var nB = vec2_multiply(n,-jB);
    if(B.inverseMass!=0)
        B.velocity.add(nB);
    if(B.inverseInertia!=0)
        B.SetOrient( B.orient + B.inverseInertia * cross_vec(rB,nB));

    if(this.joint)
    {
        // limito el angulo entre los 2 cuerpos, hardcodeado en pi/8
        var limit = Math.PI/8.0;
        var dW = A.orient-B.orient;
        if(Math.abs(dW)>limit)
        {
            // corrijo un 80% del desfasaje angular
            var desf = (Math.abs(dW)-limit) *.8;
            if(A.orient<B.orient)
                desf *= -1;

            if(A.inverseInertia!=0 )
                A.SetOrient( A.orient - desf/2);
            if(B.inverseInertia!=0 )
                B.SetOrient( B.orient + desf/2);

        }
    }
}
