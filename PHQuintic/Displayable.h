#pragma once

/*TODO: Essa defini��o foi por causa de uma refer�ncia circular
da� a defini��o da classe Graphics em foward declaration. Mas isso � coisa
de amador. Ver uma forma mais elegante de relacionar graphics com displayable.
*/
class Graphics;

//Interface para objetos visualizaveis pelo glut
class Displayable{
public:
	virtual void display(Graphics* graphic) { }
};