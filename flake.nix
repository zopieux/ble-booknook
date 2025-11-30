{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=5ae3b07d8d6527c42f17c876e404993199144b6a";
  };

  outputs = { self, nixpkgs }:
    let
      pkgs = import nixpkgs {
        system = "x86_64-linux";
        config.allowUnfree = true;
        config.segger-jlink.acceptLicense = true;
      };
      nrfutil = pkgs.nrfutil.override {
        extensions = [ "nrfutil-device" ];
      };
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        packages = [ nrfutil ] ++ (with pkgs; [ tinygo go gopls ]);
      };
    };
}
