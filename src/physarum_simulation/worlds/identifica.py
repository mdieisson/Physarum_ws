from PIL import Image

# Carrega imagem original (com canal alfa)
img = Image.open("porto23.png")

# Remove canal alfa e converte para tons de cinza
img = img.convert("L")  # "L" = modo luminância (grayscale)

img.save("porto.png")
