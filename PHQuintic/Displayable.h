#pragma once

/*TODO: Essa definição foi por causa de uma referência circular
daí a definição da classe Graphics em foward declaration. Mas isso é coisa
de amador. Ver uma forma mais elegante de relacionar graphics com displayable.
*/
class Graphics;

//Interface para objetos visualizaveis pelo glut
class Displayable{
public:
	virtual void display(Graphics* graphic) { }
};