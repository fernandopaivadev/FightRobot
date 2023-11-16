#include "Robot_Library.h"

#define RANDOM_REF 3 // pino não usado cuja leitura servirá de referência para a geração de valores aleatórios

uint8_t STATUS = SEARCH; // estado inicial: atacar
uint32_t count = 0; // contador de tempo de execução em milisegundos
uint32_t now = 0; // armazena o tempo inical em que o robô fica ocioso
uint8_t count_flag = 0; // flag para indicar se a contagem de ociosidade já foi iniciada
uint8_t search_flag = 1; // flag para indicar se a função search já foi chamada naquele ciclo de busca

void think()
{
  if(LINE_DETECT && ENEMY_DETECT)
  {
    STATUS = ESCAPE;
    count_flag = 0; // reseta a flag de contagem de ociosidade
  }
  else if (LINE_DETECT)
  {
    STATUS = ESCAPE;
    count_flag = 0; // reseta a flag de contagem de ociosidade
  }
  else if (ENEMY_DETECT)
  {
    STATUS = ATTACK;
    count_flag = 0; // reseta a flag de contagem de ociosidade
  }
  else
  {
    if ((count > (now + 1500)) && count_flag) // se ocioso durante 1.5 segundos e a contagem já foi iniciada
    {
      STATUS = SEARCH; // inicia novo ciclo de busca
      count_flag = 0; // desativa a flag de contagem de tempo ocioso
      search_flag = 0; // desativa a flag de chamada da função search(), a função será então chamada nesse ciclo
    }
    else if (!count_flag) // se a contagem ainda não foi iniciada
    {
      now = count; // inicia a contagem de tempo ocioso
      count_flag = 1; // ativa a flag de contagem de tempo ocioso
    }
  }
}

void escape()
{
  if(ALL_LINE_DETECT) // o robô está fora da arena
  {
    if(FRONT_ENEMY_DETECT)
    {
      move_foward();
    }
    else if(RIGHT_ENEMY_DETECT)
    {
      turn_right();
    }
    else if(LEFT_ENEMY_DETECT)
    {
      turn_left();
    }
    else
    {
      move_back();
    }
  }
  else if (FRONT_LINE_DETECT)
  {
    turn_right();
  }
  else if ((RIGHT_LINE_DETECT || LEFT_LINE_DETECT) && BACK_LINE_DETECT)
  {
    move_foward();
  }
  else if (RIGHT_LINE_DETECT)
  {
    turn_left();
  }
  else if (LEFT_LINE_DETECT)
  {
    turn_right();
  }
  else if (BACK_LINE_DETECT)
  {
    move_foward();
  }
}

void attack()
{
  if (FRONT_ENEMY_DETECT)
  {
    move_foward();
  }
  else if (RIGHT_ENEMY_DETECT)
  {
    turn_right();
  }
  else if (LEFT_ENEMY_DETECT)
  {
    turn_left();
  }
}

void search()
{
  if (!search_flag) // caso a função ainda não tenha sido chamada nesse ciclo de busca
  {
    random_seed(digital_read(RANDOM_REF)); // fornece um valor aleatório como referência para a função random_value()
    uint8_t value = random_value(1, 5); // gera um valor aleatório (valores possíveis: 1,2,3,4)
    switch (value)
    {
      case 1:
        turn_right();
        break;
      case 2:
        turn_left();
        break;
      case 3:
        move_foward();
        break;
      case 4:
        move_back();
        break;
    }
    // ativa a flag de chamada da função search(), essa flag evita que ela seja chamada mais de uma vez no mesmo ciclo de busca
    search_flag = 1;
  }
}

int main()
{
  _delay_ms(4500); // delay inicial de 4.5 segundos
  ir_reference(); // obter referência para os sensores
  move_foward(); // movimento inicial
  wdt_enable(WDTO_15MS); // ativa o watchdog, no caso do código travar o arduino é resetado após 15 ms

  while (1)
  {
    wdt_reset(); // reseta o watchdog para evitar que o arduino seja resetado desnecessariamente
    sense(); // realiza o sensoriamento e atualiza toas as variáveis de detecção
    think(); // determina a mudança de estados
    switch (STATUS)
    {
      case ESCAPE:
        escape();
        break;
      case ATTACK:
        attack();
        break;
      case SEARCH:
        search();
        break;
    }
    count++; // incrementa a variável de contatem de tempo
    _delay_ms(1); // incremento de tempo da variável count
  }
}
