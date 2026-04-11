function y = norm_pdf(x, mu, sigma)
    term1 = 1 / (sigma * sqrt(2 * pi));
    term2 = exp(-(x - mu).^2 / (2 * sigma^2));
    
    y = term1 * term2;
end