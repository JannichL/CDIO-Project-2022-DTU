package Solitaire.AI;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import javax.swing.JPanel;

public class Card extends JPanel {

    public int cardValue;
    public CardSuit cardSuit;
    private BufferedImage image;
    private BufferedImage backImage;
    boolean isReversed;
    Point posOffset;

    public enum CardSuit {
        Spades(1, false),
        Hearts(2, true),
        Diamonds(3, true),
        Clubs(4, false),
        Unknown(0, false);

        public int value;
        public boolean isRed;

        private CardSuit(int cardValue, boolean isRed) {
            this.value = cardValue;
            this.isRed = isRed;
        }
    };

    public static String valueInString(int value) {

        if (value == 11) {
            return "J";
        } else if (value == 12) {
            return "Q";
        } else if (value == 13) {
            return "K";
        } else if (value == 1) {
            return "A";
        }

        return Integer.toString(value);
    }

    public String toString() {
        return valueInString(cardValue) + " of " + cardSuit.name();
    }

    public String saveAsString() {
        return valueInString(cardValue) + " of " + cardSuit.name() + " of " + isReversed;
    }

    public Card(int cardValue, CardSuit cardSuit) {
        this.cardValue = cardValue;
        this.cardSuit = cardSuit;
        isReversed = false;
        File file;
        try {
            if(this.cardSuit != CardSuit.Unknown){
                file = new File("src/Solitaire/AI/images/cards/" + this.toString().toLowerCase() + ".png");
            } else {
                file = new File("src/Solitaire/AI/images/cards/unknown.png");
            }
            image = ImageIO.read(file);
            file = new File("src/Solitaire/AI/images/cards/back.png");
            backImage = ImageIO.read(file);
            setBounds(0, 0, image.getWidth(), image.getHeight());
        } catch (IOException e) {
            System.out.println("Failed to apply image to " + this);
            e.printStackTrace();
        }

        posOffset = new Point(0, 0);
        setSize(new Dimension(100, 145));
        setOpaque(false);
    }

    public void hide() {
        isReversed = true;
    }

    public void show() {
        isReversed = false;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        BufferedImage bufImg = image;
        if (isReversed) {
            bufImg = backImage;
        }

        g.drawImage(bufImg, 0, 0, this.getWidth(), this.getHeight(), null);
    }

}

