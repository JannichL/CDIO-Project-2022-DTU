
package Solitaire.AI;
import Solitaire.AI.Card.CardSuit;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

public class Deck {

    ArrayList<Card> cards;

    Boolean unknownCards = false;

    public Deck() {
        cards = new ArrayList<Card>();

        if (!unknownCards) {
            for (CardSuit suit : CardSuit.values()) {
                for (int cardValue = 1; cardValue <= 13; ++cardValue) {
                    if (suit != CardSuit.Unknown) {
                        cards.add(new Card(cardValue, suit));
                    }
                }
            }
        } else {
            for (int i = 0; i < 52; i++) {
                cards.add(new Card(i, CardSuit.Unknown));
            }
        }
        //System.out.println(cards.get(cards.size()-1));
    }

    public void shuffle() {

        int size = cards.size();
        for (int shuffles = 1; shuffles <= 20; ++shuffles) {
            for (int i = 0; i < size; i++) {
                Collections.swap(cards, i, new Random().nextInt(size));
            }
        }

    }

    public Card drawCard() {
        Card c = cards.get(0);
        cards.remove(0);
        return c;
    }

}
