#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum languages 
{
  EN,
  UA,
  DE,
  FR
} languages;

typedef struct HP_Node
{
  char title[30];
  int price;
  int pages;
  languages language;
  int weight;
  int year;

  struct HP_Node *next;
} HP_Node;

void printLanguage (languages language)
{
  switch (language)
  {
  case EN:
    printf("English ");
    break;

  case UA:
    printf("Ukarainian ");
    break;

  case DE:
    printf("Deutch ");
    break;
  
  case FR:
    printf("French ");
    break;

  default:
    printf("Language value error ");
    break;
  }
}

void push (HP_Node **head, char title[], int price, int pages, languages language, int weight, int year)
{
  HP_Node *tmp = (HP_Node*) malloc(sizeof(HP_Node));
  
  strcpy(tmp->title, title);
  tmp->price = price;
  tmp->pages = pages;
  tmp->language = language;
  tmp->weight = weight;
  tmp->year = year;

  tmp->next = (*head);
  (*head) = tmp;
}

void printHP_List(const HP_Node *head) {
    while (head) {
        printf("Title: %s ", head->title);
        printf("Price: %d hrn ", head->price);
        printf("Language: ");
        printLanguage(head->language);
        printf("Weight: %d gram ", head->weight);
        printf("Publish year: %d ", head->year);
        printf("\n");
        head = head->next;
    }
    printf("\n");
}

void printNthHP_List(HP_Node* head, int nth)
{
  for (int i = 0; i < nth; ++i)
  {
    head = head->next;
  }

  printf("Title: %s ", head->title);
  printf("Price: %d hrn ", head->price);
  printf("Language: ");
  printLanguage(head->language);
  printf("Weight: %d gram ", head->weight);
  printf("Publish year: %d ", head->year);
  printf("\n");
}

int main()
{
  HP_Node *head = NULL;

  push(&head, "HP_1", 550, 330, EN, 230, 2002);
  push(&head, "HP_2", 430, 400, UA, 320, 2003);
  push(&head, "HP_3", 620, 475, FR, 330, 2004);
  push(&head, "HP_4", 355, 600, DE, 290, 2005);

  printHP_List(head);
  printHP_List(head);
  printNthHP_List(head, 0);
  printNthHP_List(head, 3);
  return 0;
}
