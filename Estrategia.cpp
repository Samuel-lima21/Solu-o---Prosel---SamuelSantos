#include "Estrategia.h"
#include <cmath>
#include <vector>

using namespace std;

//define a estrutura do Vector
struct Vector {                                         
    double x, y;

    Vector(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}

    //define a soma de vetores
    Vector operator + (const Vector& v) const {        
        return Vector(x + v.x, y + v.y);
    }

    //define a subtração
    Vector operator - (const Vector& v) const {         
        return Vector(x - v.x, y - v.y);
    }

    //define a operação da multiplicação por um escalar
    Vector operator * (double s) const {                
        return Vector(x * s, y * s);
    }
};

struct Obstacle {
    Vector pos;
    Vector vel;     
};

//calcula a magnitude de um vetor (norma)
double norm(const Vector& v){                           
	return std::sqrt(v.x*v.x + v.y*v.y);
}

//normaliza um vetor (x/|Vector|, y/|Vector|)
Vector normalizar(const Vector& v) {                    
    double n = norm(v);
    if (n == 0) return Vector(0.0, 0.0);
    return Vector(v.x / n, v.y / n);
}

//calcula a posição futura do obstáculo com base na posição atual, velocidade e um parâmetro de variação (t)
Vector posicao_obstaculo(const Vector& pos, const Vector& vel, double t) {   
    return pos + vel * t;                                                   
}

bool alinhado_com_gol(const Vector& robo, const Vector& gol, const Vector& bola){
    Vector robo_gol = normalizar(gol - robo);
    Vector robo_bola = normalizar(bola - robo);
    double produto_escalar = robo_gol.x * robo_bola.x + robo_gol.y * robo_bola.y;
    return produto_escalar > 0.6;
}

Vector calcula_posicao_alinhada(const Vector& robo, const Vector& gol, const Vector& bola){
    Vector robo_gol = normalizar(gol - robo);
    Vector robo_bola = normalizar(bola - robo);
    Vector perp(-robo_bola.y, robo_bola.x);

    double produto = robo_gol.x * perp.x + robo_gol.y * perp.y; 

    double distancia = 0.5;
    return bola + perp*distancia; 
}

//Força de atração - tem como parâmtros a posição atual do meu robô, a posição do objetivo
//e uma constante de atração
Vector ForcaAtrativa(const Vector& robo, const Vector& goal, double k_att) {
    return (goal - robo) * k_att;
}

//Força de Repulsão - tem como parâmetros a posição atual do meu robô, as informações do obstáculo (posição e velocidade),
//uma constante de repulsão, o raio de ação e o parâmetro de variação t
Vector ForcaRepulsiva(const Vector& robo, const Obstacle& obs, double k_rep, double raio_acao, double t) {
    //calcula a posição futura
    Vector PosicaoFutura = posicao_obstaculo(obs.pos, obs.vel, t);
    
    //calcula a distância entre o robo e a posição do obstáculo
    Vector distanciaObstaculo = robo - PosicaoFutura;
    double d = norm(distanciaObstaculo);

    //verifica se está dentro do raio de ação
    if (d > raio_acao || d == 0){
        return Vector(0, 0);
    } else{
    //calcula o valor da força de repulsão e multiplica pelo vetor distanciaObstaculo
        double repulsao = k_rep * (1.0/d - 1.0/raio_acao) / (d * d);
        return (normalizar(distanciaObstaculo)) * repulsao;
    }

}

//Calcula uma força de vórtice para o robô sair do mínimo local
Vector ForcaVortice(const Vector& robo, const Obstacle& obs, const Vector& goal, double k_vort, double raio_acao, double t) {
    Vector PosicaoFutura = posicao_obstaculo(obs.pos, obs.vel, t); 
    //calcula a distância entre o robo e a posição do obstáculo
    Vector distanciaObstaculo = robo - PosicaoFutura;
    double d = norm(distanciaObstaculo);

    if (d > raio_acao || d == 0){
        return Vector(0,0);
    }else{
        // direção para o objetivo
        Vector toGoal = normalizar(goal - robo);
        // vetor perpendicular à ao vetor distânciaObstáculo
        Vector perp(-distanciaObstaculo.y, distanciaObstaculo.x);
        //Produto escalar
        // Se produto_escalar = 0, os vetores são perpendiculares 
        // Se produto_escalar > 0, os vetores toGoal e perp estão apontando para o mesma direção
        double produto_escalar = perp.x * toGoal.x + perp.y * toGoal.y;
        //verifica se estão apontando para direções opostas. Se sim, inverte o vetor perp
        if (produto_escalar < 0) {
            perp = perp * -1.0;
        }
        perp = normalizar(perp);
        //calcula a intensidade da força de vórtice
        double intensidade = k_vort * (1.0/d - 1.0/raio_acao);
        return perp * intensidade;
    }
    
}

Estrategia::Estrategia(int id, bool isTeamA) : id(id), teamA(isTeamA) {
    if (id == 0) role = "Goleiro";
    else if (id == 1) role = "Ala";
    else role = "Atacante";
}

Action Estrategia::think(const GameState& state) {
    Action a;
    a.moveDirectionX = 0;
    a.moveDirectionY = 0;

    // TODO: Implementar lógica de tomada de decisão e planejamento aqui!
    // Você tem acesso a:
    // - state.ball (bola)
    // - state.getMe() (seu robô)
    // - state.teammates (aliados)
    // - state.opponents (inimigos)
    // - state.myIndex (seu ID: 0, 1 ou 2)

    const EntityState& eu = state.getMe();
    const EntityState& bola = state.ball;

    //calcular a distância do robô até a bola
    float dist_bola = sqrt((eu.x - bola.x)*(eu.x - bola.x) + (eu.y - bola.y)*(eu.y - bola.y));

    // estratégia do goleiro
    if (role == "Goleiro") {
        float meuGolX; 
        float alvoY = bola.y;                       //O goleiro vai acompanhar a posição y da bola

        if (teamA){
            meuGolX = -0.75f;                       //Define a posição de cada goleiro
        }else{
            meuGolX = 0.75f;
        }

        if (dist_bola > 0.068){

            if (alvoY > 0.20f) alvoY = 0.20f;           //fica limitado em y entre 0.20 e -0.20
            if (alvoY < -0.20f) alvoY = -0.20f;
         
        }else{
            if (teamA){
                meuGolX = meuGolX + 0.15;               //Dá um "empurrãozinho" na bola para frente
            }else{
                meuGolX = meuGolX - 0.15;
            }
        }

        float angulo = eu.angleTo(meuGolX, alvoY);
        a.moveDirectionX = std::cos(angulo);
        a.moveDirectionY = std::sin(angulo);
          

        return a;
    }

    if (teamA){
        //Estratégia do Atacante
        if (role == "Atacante"){

            std::vector<Vector> posicao_jog;                            //inicializa o Vector
            std::vector<Obstacle> obstacles;                            //inicializa os obstáculos
            Vector gol = teamA ? Vector(0.75, 0.0) : Vector(-0.75, 0.0); //vetor posição do gol

            posicao_jog.push_back(Vector{Vector(eu.x, eu.y)}); // adiciona ou Vector o vetor(eu.x, eu.y)
            

            for (const auto& op : state.opponents) { //itera sobre o state.opponents armazenando a posição e velocidade
                obstacles.push_back(Obstacle{Vector(op.x, op.y), Vector(op.vx, op.vy)});
            } 

            Vector F = Vector(0,0); //inicializa o vetor Força
            Vector bola_1 = Vector(bola.x, bola.y);//inicializa o vetor bola
            Vector alvo;

            if (alinhado_com_gol(posicao_jog[0], gol, bola_1)){
                if (dist_bola > 0.065) {
                    // longe da bola → vai pra bola
                    alvo = bola_1;
                    //calcula a o vetor força de atração para a bola
                    F = F + ForcaAtrativa(posicao_jog[0], alvo, 90.0);
                } else {
                    // perto da bola → vai pro gol
                    alvo = gol; 
                    //calcula a o vetor força de atração para o gol
                    F = F + ForcaAtrativa(posicao_jog[0], alvo, 90.0);
                }   
            }else{
                Vector alvo_alinhamento = calcula_posicao_alinhada(posicao_jog[0], gol, bola_1);
                alvo = bola_1;
                F = F + ForcaAtrativa(posicao_jog[0], alvo, 100.0);
                F = F + ForcaAtrativa(posicao_jog[0], alvo_alinhamento, 100.0);
            }
            

            //calcula o força repulsiva e de vórtice exercida por cada opponente
            for (const auto& op : state.opponents) {
                Obstacle obs{Vector(op.x, op.y), Vector(op.vx, op.vy)};
                F = F + ForcaRepulsiva(posicao_jog[0], obs, 110.0, 0.15, 0.4);
                F = F + ForcaVortice(posicao_jog[0], obs, alvo, 200.0, 0.22, 0.2);
            }

            double max_F = 20.0;
            if (norm(F)>max_F){
                F = normalizar(F)*max_F;
            }

            Vector destino = posicao_jog[0] + F;
            float angulo = eu.angleTo(destino.x, destino.y);
            a.moveDirectionX = std::cos(angulo);
            a.moveDirectionY = std::sin(angulo);
            
        //estrategia do ala    
        } else if (role == "Ala"){

            if (bola.y < 0){
                //fica na diagonal da bola esperando o passe
                float alvoX = bola.x + 0.20f;
                float alvoY = bola.y + 0.20f;
                float angulo = eu.angleTo(alvoX, alvoY);
                a.moveDirectionX = std::cos(angulo);
                a.moveDirectionY = std::sin(angulo);
            } else{
                float alvoX = bola.x + 0.20f;
                float alvoY = bola.y - 0.20f;
                float angulo = eu.angleTo(alvoX, alvoY);
                a.moveDirectionX = std::cos(angulo);
                a.moveDirectionY = std::sin(angulo);
            }
        }
        
    }  else {
        if (role == "Atacante"){   //atua como um "volante"
            float alvoX = 0.4f;
            float alvoY = bola.y;

        if (dist_bola > 0.072){    //Se a bola está longe, fica acompanhando no eixo y
            float angulo = eu.angleTo(alvoX, alvoY);
            a.moveDirectionX = std::cos(angulo);
            a.moveDirectionY = std::sin(angulo); 
        }else{                     //Se está próxima, empurra para longe
            alvoX = alvoX - 0.15;                       
            float angulo = eu.angleTo(alvoX, alvoY);
            a.moveDirectionX = std::cos(angulo);
            a.moveDirectionY = std::sin(angulo);
        }

            

        } else if(role == "Ala") { //lateral defensivo
            float alvoX = bola.x + 0.15f;
            float alvoY = bola.y + 0.15f; 

            if (bola.x > 0.0){
                if (dist_bola > 0.072){                         //Se a bola está longe, fica acompanhando no eixo y
                    float angulo = eu.angleTo(alvoX, alvoY);
                    a.moveDirectionX = std::cos(angulo);
                    a.moveDirectionY = std::sin(angulo); 
                }else{                                          //Se está próxima, empurra para longe
                    alvoX = alvoX - 0.15;                       
                    float angulo = eu.angleTo(alvoX, alvoY);
                    a.moveDirectionX = std::cos(angulo);
                    a.moveDirectionY = std::sin(angulo);
                }
            }           
        }
    }
    return a;
}