
/**
 * @fileoverview contact.js

<P> Collision response (impulse resolution)</P>
<P> Los motores de fisicas usualmente tienen 2 etapas bien definidas</P>
<P> * Etapa de Collision Detection: el objetivo de esta etapa es determinar si 2 o mas cuerpos entran en colision. </P>
<P> PAra ello se aplican una serie de algoritmos geometricos bastante estudiados. </P>
<P> Esta etapa a su vez se puede dividir en 2 fases :</P>
<P>              - Broad Phase. </P>
<P>	Se trata de rapidamente descartar aquellos pares de cuerpos que por estar lo suficienemente
                              alejados no pueden entrar en colision. Se suelen usar bounding box y otro tipo de simplificaciones
                              y distintas estructuras jerarquicas. La idea es que rapidamente se puedan detectar pares de 
                              candidatos que pueden entrar en colision.</P>
							  
<P>              - Narrow Phase. </P>
<P>Con el resultado de pares "posibles" de la fase anterior, se determinar exactamente si 
                              esos "candidatos" a colision, realmente estaban colisionando, y en caso afirmativo
                              colectar informacion especifica de la colision, que luego va a servir en etapas posteriores
                              como el punto exacto de la colision, la normal, y la penetracion.</P>
							  
<P> * Etapa de Collision Response : una vez dectectada la collision, el motor tiene que tomar alguna acción para evitar
 que los cuerpos que collisionaron siguen en esa situacion. En nuestro caso se resuelve aplicando un impulso de tal forma
 que los cuerpos se alejen en la direccion que chocaron,  y asi evitar que sigan en colision = Impulse Resolution.</P>
<P> A continuacion se codifican los algortimos de Impulse Resolution.</P>
<P> Impulse Resolution es una de las tantas formas estandard de los motores de fisica de resolver la colision entre cuerpos
 La idea es aplicar un impulso (cambio instantaneo en la velocidad) a los cuerpos que entran en colision, de tal manera
 que tienan a separarse. </P>
<P>Los algoritmos que detectan la colision tambien informan la normal de la colision, el punto de contacto y la penetracion</P>
*/

// 
/**
 * @class ContactSolver contactos entre 2 cuerpos
 * @property {Body} A cuerpo A 
 * @property {Body} B cuerpo B 
 * @property {float} penetracion  Cuanto penetra el Cuerpo A dentro del B durante el contacto
 * @property {Vector2} normal Vector de A to B
 * @property {Vector2[]} contacts Array de Puntos de contacto. 
 * @property {int} contact_count Cantidad de puntos de contacto
 * @property {float} dt elapsed time
 * @property {float} e Coeficiente de restitution
 * @property {float} df Coeficiente de friccion dinamica
 * @property {float} sf Coeficiente de friccion estatica
 * @property {boolean} resting_contact flag que indica que es un contacto tipo "resting", por ejemplo un libro apoyado sobre una mesa
 * @constructor
 */

function ContactSolver()
{
    this.A = null;                      // cuerpo A
    this.B = null;                      // cuerpo B
    this.penetration = 0;			    // Cuanto penetra el Cuerpo A dentro del B durante el contacto
    this.normal = new Vector2(0,0);		// Vector de A to B
    this.contacts = [];		            // Puntos de contacto. 
    this.contact_count = 0;			    // Cantidad de puntos de contacto
    this.dt = 0;					    // elapsed time
    this.e = 0;					        // Coeficiente de restitution
    this.df = 0;					    // Coeficiente de friccion dinamica
    this.sf = 0;					    // Coeficiente de friccion estatica
    this.resting_contact = false;
}

// Los algoritmos de colision propiamente dichos se implementan en el collision.js

/**
 * 
 * @method BodyCollide Determina si hay contacto entre el cuerpo A y el Cuerpo B
 * @param {float} elapsed_time tiempo transcurrido
 * @param {Body} A body A
 * @param {Body} B body B
*/

ContactSolver.prototype.BodyCollide = function(elapsed_time, pA , pB)
{
    this.dt = elapsed_time;
    this.A = pA;
    this.B = pB;
    this.contact_count = 0;

    if(pA.shape.GetType()==GPE_CIRCLE && pB.shape.GetType()==GPE_CIRCLE)
        CircleCircle(this.A, this.B, this);
    else
    if(pA.shape.GetType()==GPE_CIRCLE && pB.shape.GetType()==GPE_POLYGON)
        CirclePolygon(this.A, this.B, this);
    else
    if(pA.shape.GetType()==GPE_POLYGON && pB.shape.GetType()==GPE_CIRCLE)
    {
        this.A = pB;
        this.B = pA;
        CirclePolygon(this.A, this.B, this);
    }
    else
    if (pA.shape.GetType() == GPE_POLYGON && pB.shape.GetType() == GPE_POLYGON)
        PolygonPolygon(this.A, this.B, this);
    
    return this.contact_count > 0 ?  true : false;
}


// El cuerpo A y el B entraron en colision, precomputa ciertos parametros para resolver luego el impulso
// La idea es modificar la velocidad de los cuerpos para que se alejan en direccion a la normal donde hicieron contacto
// De esta forma se evitara que sigan colisionando
ContactSolver.prototype.PreCalc = function ()
{
    // El coeficiente de restitucion indica que tan elastico es el choque, o cuanta energia se pierde por el impacto
    // usualmente se toma el minimo entre los 2 cuerpos que entran en colision,
	// pero yo voy a tomar el maximo
    this.e = Math.max( this.A.restitution, this.B.restitution );

    // Computo los coeficientes de friccion (Ley de Coulumb)
    this.sf = Math.sqrt( this.A.staticFriction * this.B.staticFriction );
    this.df = Math.sqrt( this.A.dynamicFriction * this.B.dynamicFriction );

    for(var i = 0; i < this.contact_count; ++i)
    {

        // la velocidad de un punto en el cuerpo rigido se puede computar como la velocidad de su centro de masas
        // producto del movimiento lineal del cuerpo, mas la vecocidad tangencial, producto de su rotacion
        // Velocidad tangencial depende de la distancia al centro de masas
        // Vt = w x r 

        // Computo los radios desde el pto de contacto hasta el centro de masa 
        var ra = vec2_substract(this.contacts[i], this.A.position);
        var rb = vec2_substract(this.contacts[i], this.B.position);

        // computo la velocidad de los puntos de contacto 
        var va = vec2_add(this.A.velocity, cross(this.A.angularVelocity, ra));
        var vb = vec2_add(this.B.velocity, cross(this.B.angularVelocity, rb));

        // Computa la velocidad relativa. Es como si uno de los cuerpos estuviera fijo, y el otro se mueve con
        // la velocidad total del sistema. Es un cambio de referencia. 
        var rv = vec2_substract(vb , va);

        // Verifico si se trata de "resting contact" 
        // Se llama resting contact a los contactos en equilibrio estatico, por ejemplo un libro sobre la mesa
        // la idea es que si la unica fuerza que acelera el objeto es la de gravedad
        // la colision se tiene que hacer sin ninguna restitucion, para evitar que el cuerpo rebote indefinidamente
        if (rv.lengthSqr() < gravity.lengthSqr() * this.dt * this.dt + EPSILON)
        {
            this.e = 0;
        }
            
    }

    /*
    // parche para saber si es un resting contact estatico.
    // si el cuerpo esta en equilibrio estatico
    if(this.contact_count>=2 && this.e==0 && (this.A.inverseMass==0 || this.B.inverseMass==0))
    {
        this.resting_contact = true;
    }
    */
}

// Resolver el impulso 
ContactSolver.prototype.ApplyImpulse = function()
{
    // si ambos objetos son esticos, corrijo la posicion directamente
    if(this.A.inverseMass  + this.B.inverseMass < EPSILON)
    {
        this.staticBodyCorrection();
        return;
    }

    if (this.resting_contact)
    {
        this.staticBodyCorrection();
        return;
    }


    for(var i = 0; i < this.contact_count; ++i)
    {
        // la velocidad de un punto en el cuerpo rigido se puede computar como la velocidad de su centro de masas
        // producto del movimiento lineal del cuerpo, mas la vecocidad tangencial, producto de su rotacion
        // Velocidad tangencial depende de la distancia al centro de masas
        // Vt = w x r 

        // Computo los radios desde el pto de contacto hasta el centro de masa 
        var ra = vec2_substract(this.contacts[i], this.A.position);
        var rb = vec2_substract(this.contacts[i], this.B.position);

        // la velocidad de un punto en el cuerpo rigido se puede computar como la velocidad de su centro de masas
        // producto del movimiento lineal del cuerpo, mas la vecocidad tangencial, producto de su rotacion
        var va = vec2_add(this.A.velocity, cross(this.A.angularVelocity, ra));
        var vb = vec2_add(this.B.velocity, cross(this.B.angularVelocity, rb));

        // Computa la velocidad relativa. Es como si uno de los cuerpos estuviera fijo, y el otro se mueve con
        // la velocidad total del sistema. Es un cambio de referencia. 
        var rv = vec2_substract(vb , va);

        // se llama velocidad del contacto al componente de la velocidad sobre la normal del mismo. 
        // Es la parte de la misma que interviene en la restitucion.
        var contactVel = dot( rv, this.normal );

        // Luego, si es velocidad es positiva los objetos ya se estan separando, no tengo que hacer nada
        if(contactVel > 0)
            return;

        // Para corregir la orientacion del cuerpo, preciso trabajar con el momento de la fuerza con respecto al punto
        // Eso indica que parte de la fuerza produce una acelaracion angular, o interviene en la rotacion.,
        var raCrossN = cross_vec( ra, this.normal );				// momento de A
        var rbCrossN = cross_vec( rb, this.normal );				// momento de B
        var invMassSum = this.A.inverseMass + this.B.inverseMass 
			+ raCrossN*raCrossN * this.A.inverseInertia + rbCrossN*rbCrossN * this.B.inverseInertia;

        // Ecuacion para el impuslo escalar j
        // notar que el impuso es invesamente proporcional a la masa
        // ej. Si un camion choca contra una bicicleta, la bicicleta sale disperando y el cambio apenas se mueve
        var j = -(1 + this.e) * contactVel;
        j /= invMassSum;
        j /= this.contact_count;

        // Aplico el impulso pp dicho
        this.A.ApplyImpulse(vec2_multiply(this.normal, -j), ra);
        this.B.ApplyImpulse(vec2_multiply(this.normal, j), rb);

        if (this.resting_contact) {
            this.staticBodyCorrection();
        }
        
        if (!this.resting_contact) {

            // Computo la fuerz de friccion, la fuerza es tangente la direccion de colision, es decir la direccion de colision
            // es la normal, y la friccion es la tangente en el choque. 
            // vuelvo a computar la velocidad relativa (luego de haber corregido las mismas con los impulsos que aplicamos )
            va = vec2_add(this.A.velocity, cross(this.A.angularVelocity, ra));
            vb = vec2_add(this.B.velocity, cross(this.B.angularVelocity, rb));
            rv = vec2_substract(vb, va);
            var t = vec2_substract(rv, vec2_multiply(this.normal, dot(rv, this.normal)));
            if (t.lengthSqr() == 0)
                return;

            t.normalize();
            // j tangent magnitude
            var jt = -dot(rv, t);
            jt /= invMassSum;
            jt /= this.contact_count;


            // si la frccion es muy pequeña no la tengo en cuenta para no introducir inestabilidad en el sistmea
            if (Math.abs(jt) < 0.01)
                return;

            // Ley de Coulumb's para la friccion
            // hay 2 tipos de friccion, 
            // estatica, es la la fuerza de resitencia que impone un objeto estatico a ser movido. 
            // dinamica, idem cuando el cuerpo se esta movimiendo
            // por ejemplo un libro sobre la mesa esta quito y si quiero moverlo, la friccion que tiene impide que el libro
            // se mueva es la friccion estatica. Para mover el libro tengo que hacer una fuerza superior al la de friccion estatica
            // asi el libro se empieza a mover. A partir de ese momento, el libro ya esta en movimiento, y comienza a actuar 
            // la friccion dinamica. 
            var tangentImpulse = t;
            if (Math.abs(jt) < j * this.sf)
                tangentImpulse.multiply(jt);
            else
                tangentImpulse.multiply(-j * this.df);

            // Aplico el impulso de la friccion
            this.A.ApplyImpulse(vec2_multiply(tangentImpulse, -1), ra);
            this.B.ApplyImpulse(tangentImpulse, rb);
        }
        // paso al siguiente contacto
    }

}

// Parche para corregir la penetration
ContactSolver.prototype.positionCorrection = function ()
{
    var k_slop = 0.05; // Penetration allowance
    var percent = 0.4; // Penetration percentage to correct
    var K = (Math.max(this.penetration - k_slop, 0) / (this.A.inverseMass + this.B.inverseMass)) * percent;
    var correction = vec2_multiply(this.normal, K);
    this.A.position.substract(vec2_multiply(correction , this.A.inverseMass));
    this.B.position.add(vec2_multiply(correction , this.B.inverseMass));
}


// Parche para corregir los objetos estaticos
ContactSolver.prototype.staticBodyCorrection = function()					
{
    // simplemente elimino la velocidad.
    this.A.velocity.Set(0,0);
    this.B.velocity.Set(0,0);
}

